use core::cmp::{min, Ordering};
use defmt::{info, warn};
use fdcan::config::FrameTransmissionConfig::ClassicCanOnly;
use fdcan::config::Interrupts;
use fdcan::id::{Id, StandardId};
use heapless::binary_heap::Max;
use heapless::BinaryHeap;
use rtic_sync::{channel, make_channel};
use fdcan::{self, Fifo0, NormalOperationMode, ReceiveOverrun};
use fdcan::{
    config::NominalBitTiming,
    filter::{StandardFilter, StandardFilterSlot},
    frame::{RxFrameInfo, TxFrameHeader},
};
use can_bit_timings::CanBitTiming;

// Module to support software TX and RX queued CAN on rtic
//
// TX side uses a binary heap to send out messages in priority order.
// RX side writes messages into an RTIC channel for processing by the app.
//
// Currently not very abstract: uses STM32G4 FDCAN peripheral directly,
// hard-codes CAN baud rate, etc.


// CAN TX and RX software queue sizes
const RX_CAPACITY: usize = 8;
const TX_CAPACITY: usize = 16;

// Types for each end of the Rx rtic channel
pub type RxReceiver = channel::Receiver<'static, QueuedFrame, RX_CAPACITY>;
type RxSender = channel::Sender<'static, QueuedFrame, RX_CAPACITY>;

pub fn init<I: fdcan::Instance>(mut can: fdcan::FdCan<I, fdcan::ConfigMode>, bit_timings: &CanBitTiming) -> (fdcan::FdCanControl<I, fdcan::NormalOperationMode>, Rx<I>, RxReceiver, TxQueue<I>)
{
    // Convert the generic bit timings to FDCAN bit timings
    let btr = NominalBitTiming {
        prescaler: bit_timings.prescaler.try_into().unwrap(),
        seg1: bit_timings.bs1.try_into().unwrap(),
        seg2: bit_timings.bs2.try_into().unwrap(),
        sync_jump_width: bit_timings.sjw.try_into().unwrap(),
    };

    can.set_protocol_exception_handling(false);
    can.set_nominal_bit_timing(btr);
    can.set_standard_filter(
        StandardFilterSlot::_0,
        StandardFilter::accept_all_into_fifo0(),
    );
    can.set_frame_transmit(ClassicCanOnly); // Currently no FD long frame support
    can.enable_interrupts(Interrupts::RX_FIFO0_NEW_MSG | Interrupts::ERR_PASSIVE | Interrupts::BUS_OFF | Interrupts::TX_COMPLETE);
    can.select_interrupt_line_1(Interrupts::TX_COMPLETE);
    info!("-- Current Config: {:#?}", can.get_config());
    let can = can.into_normal();

    //nb::block!(can.enable_non_blocking()).unwrap(); // TODO

    let (can_control, can_tx, can_rx, _can_rx1) = can.split();

    let (rx, rx_receiver) = Rx::new(can_rx);

    (can_control, rx, rx_receiver, TxQueue::new(can_tx))
}

#[derive(Clone, Debug)]
pub struct QueuedFrame {
    pub header: TxFrameHeader, // Note: used for TX and RX direction
    pub data: [u8; 8], // Fixed size array, see header.len for 'real' length
}

// TODO: Figure out how to use fdcan::frame::FramePriority here, currently
// the implementation is copied from fdcan id.rs file
impl Ord for QueuedFrame {
    fn cmp(&self, other: &Self) -> Ordering {
        let id_a = self.header.id;
        let id_b = other.header.id;
        match (id_a, id_b) {
            (Id::Standard(a), Id::Standard(b)) => {
                // Lower IDs have priority over higher IDs.
                a.as_raw().cmp(&b.as_raw()).reverse()
            }
            (Id::Extended(a), Id::Extended(b)) => {
                a.as_raw().cmp(&b.as_raw()).reverse()
            }
            (Id::Standard(a), Id::Extended(b)) => {
                // Standard frames have priority over extended frames if their Base IDs match.
                a.as_raw()
                    .cmp(&b.standard_id().as_raw())
                    .reverse()
                    .then(Ordering::Greater)
            }
            (Id::Extended(a), Id::Standard(b)) => a
                .standard_id()
                .as_raw()
                .cmp(&b.as_raw())
                .reverse()
                .then(Ordering::Less),
        }
    }
}

impl PartialOrd for QueuedFrame {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

// This seems wrong!
impl PartialEq for QueuedFrame {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other) == Ordering::Equal
    }
}

impl QueuedFrame {
    // Create a new QueuedFrame with a standard CAN ID
    pub fn new_std(id_raw: u16, data: &[u8]) -> Self {
        let id = StandardId::new(id_raw).unwrap();
        assert!(data.len() <= 8);
        Self::new_tx(TxFrameHeader {
            len: data.len() as u8,
            frame_format: fdcan::frame::FrameFormat::Standard,
            id: Id::Standard(id),
            bit_rate_switching: false,
            marker: None,
        }, data)
    }

    // Internal constructor for queue receiver
    fn new_rx(info: &RxFrameInfo, data: &[u8]) -> Self {
        Self::new_tx(info.to_tx_header(None), data) 
    }

    // Internal constructor for re-queue on transmit
    fn new_tx(header: TxFrameHeader, tx_data: &[u8]) -> Self {
        let mut data = [0_u8; 8];
        let dlen = min(header.len, 8) as usize;
        data[..dlen].copy_from_slice(&tx_data[..dlen]);
        Self {
            header,
            data,
        }
    }

    // Function to pass to transmit_pending(), dequeues a pending
    // message as QueuedFrame
    fn from_pending_transmit(_: fdcan::Mailbox, header: TxFrameHeader, data32: &[u32]) -> Self {
        // Awkward workaround for fdcan passing these values as &[u32]
        let mut data = [0_u8; 8];
        let dlen = min(min(header.len as usize, core::mem::size_of_val(data32)), 8);
        unsafe {
            data[..dlen].copy_from_slice(&data32.align_to::<u8>().1[..dlen]);
        }
        QueuedFrame{ header, data }
    }
}

impl Eq for QueuedFrame {}

pub struct TxQueue<I: fdcan::Instance> {
    can: fdcan::Tx<I, NormalOperationMode>,
    queue: BinaryHeap<QueuedFrame, Max, TX_CAPACITY>,
}

pub trait Tx {
    fn transmit(&mut self, msg: &QueuedFrame);
}

impl<I: fdcan::Instance> TxQueue<I> {
    fn new(can: fdcan::Tx<I, NormalOperationMode>) -> Self {
        Self {
            can,
            queue: BinaryHeap::new(),
        }
    }

    pub fn on_tx_irq(&mut self) {
        if let Some(msg) = self.queue.pop() {
            self.transmit(&msg);
        }
    }
}

impl<I: fdcan::Instance> Tx for TxQueue<I> {
    fn transmit(&mut self, msg: &QueuedFrame) {
        let maybe_queue = match self.can.transmit_preserve(msg.header, &msg.data,
            &mut QueuedFrame::from_pending_transmit) {
                // Preserve the pending TX message that was replaced in hardware
                Ok(Some(dequeued)) => Some(dequeued),
                // Rather than blocking on fdcan, queue this message for later transmit
                Err(nb::Error::WouldBlock) => Some(msg.clone()),
            _ => None,
        };
        if let Some(to_queue) = maybe_queue {
            match self.queue.push(to_queue) {
                Ok(_) => (),
                _ => {
                    defmt::warn!("TX queue overflow");
                    // Generally all the data we send is only useful if fresh, so
                    // clear the transmit queue if it seems like the bus is offline
                    self.queue.clear();
                }
            }
        }
    }
}

// Currently only supports receiving from FIFO0
pub struct Rx<I: fdcan::Instance> {
    can: fdcan::Rx<I, NormalOperationMode, Fifo0>,
    rx_sender: RxSender,
}

impl<I: fdcan::Instance> Rx<I> {
    fn new(can: fdcan::Rx<I, NormalOperationMode, Fifo0>) -> (Self, RxReceiver) {
        let (sender, receiver) = make_channel!(QueuedFrame, RX_CAPACITY);
        (
            Self {
                can,
                rx_sender: sender,
            },
            receiver,
        )
    }

    pub fn on_rx_irq(&mut self) {
        let mut buffer = [0_u8; 8];
        let rx_header = match self.can.receive(buffer.as_mut_slice()) {
            Ok(ReceiveOverrun::NoOverrun(header)) => header,
            Ok(ReceiveOverrun::Overrun(header)) => {
                // I think this only happens if the buffer is too small, so
                // in Classic CAN it shouldn't ever.
                warn!("CAN RX overrun reported");
                header
            }
            // Shouldn't happen unless RX IRQ fired without anything received
            Err(err) => panic!("CAN internal error {err:?}"),
        };
        let frame = QueuedFrame::new_rx(&rx_header, &buffer);
        match self.rx_sender.try_send(frame) {
            Ok(_) => (),
            // Can probably leave this as panic for now, as app should be able to handle max incoming CAN rate
            Err(err) => panic!("CAN RX queue overflow {err:?}"),
        }
    }
}

