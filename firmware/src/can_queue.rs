//! Module to support software TX and RX queued CAN on rtic
//!
//! TX side uses a binary heap to send out messages in priority order.
//! RX side writes messages into an RTIC channel in FIFO order, for processing by the app.
use can_bit_timings::CanBitTiming;
use embedded_can::{Frame, Id, StandardId};
use core::cmp::{min, Ordering};
use fdcan::config::FrameTransmissionConfig::ClassicCanOnly;
use fdcan::config::InterruptLine;
use fdcan::interrupt::{Interrupt, Interrupts};
use fdcan::{self, Fifo0, Mailbox, NormalOperationMode, ReceiveOverrun};
use fdcan::{
    config::NominalBitTiming,
    filter::{StandardFilter, StandardFilterSlot},
    frame::{FramePriority, TxFrameHeader},
};
use heapless::binary_heap::Max;
use heapless::BinaryHeap;
use rtic::Mutex;
use rtic_sync::{channel, make_channel};

/// Software RX queue size
const RX_CAPACITY: usize = 16;
/// Software TX queue size
const TX_CAPACITY: usize = 32;

// Types for each end of the Rx rtic channel

/// The receive end is the public interface to receive CAN frames
pub type Rx = channel::Receiver<'static, QueuedFrame, RX_CAPACITY>;

/// The send end is only for internal use
type RxSender = channel::Sender<'static, QueuedFrame, RX_CAPACITY>;

/// Control struct is used when instantiating the queue, and
/// by the interrupt handler function
pub struct Control<I: fdcan::Instance> {
    hw: fdcan::FdCanControl<I, fdcan::NormalOperationMode>,
    hw_rx: fdcan::Rx<I, NormalOperationMode, Fifo0>,
    rx_sender: RxSender,
}

impl<I: fdcan::Instance> Control<I> {
    pub fn init(
        mut can: fdcan::FdCan<I, fdcan::ConfigMode>,
        bit_timings: &CanBitTiming,
    ) -> (Self, Rx, Tx<I>) {
        // Convert the generic bit timings to FDCAN bit timings
        defmt::debug!(
            "CAN prescaler {} bs1 {} bs2 {} sjw {}",
            bit_timings.prescaler,
            bit_timings.bs1,
            bit_timings.bs2,
            bit_timings.sjw
        );
        let btr = NominalBitTiming {
            prescaler: bit_timings.prescaler.try_into().unwrap(),
            seg1: bit_timings.bs1.try_into().unwrap(),
            seg2: bit_timings.bs2.try_into().unwrap(),
            sync_jump_width: bit_timings.sjw.try_into().unwrap(),
        };

        can.set_nominal_bit_timing(btr);
        // Currently only supports receiving into FIFO0, assume software can keep up
        can.set_standard_filter(
            StandardFilterSlot::_0,
            StandardFilter::accept_all_into_fifo0(),
        );
        can.set_frame_transmit(ClassicCanOnly); // Currently no FD long frame support

        can.enable_interrupt_line(InterruptLine::_1, true); // Swapped in crate, this is line 0
        can.enable_interrupts(
            Interrupts::RX_FIFO0_NEW_MSG
                | Interrupts::ERR_PASSIVE
                | Interrupts::BUS_OFF
                | Interrupts::TX_COMPLETE,
        );
        //can.enable_transmission_interrupts(Mailboxes::all());
        defmt::info!("Configuring fdcan...");

        // Make the RTIC channel for received messages
        let (rx_sender, rx_receiver) = make_channel!(QueuedFrame, RX_CAPACITY);

        // Start the CAN peripheral and split it
        let (hw, hw_tx, hw_rx, _hw_rx1) = can.into_normal().split();

        (
            Self {
                hw,
                hw_rx,
                rx_sender,
            },
            rx_receiver,
            Tx::new(hw_tx),
        )
    }

    pub fn on_irq<M>(&mut self, mut m_tx: M)
    where
        M: Mutex<T = Tx<I>>,
    {
        // This is kind of annoying that we have to poll the
        // interrupt register on each check
        if self.hw.has_interrupt(Interrupt::RxFifo0NewMsg) {
            self.hw.clear_interrupt(Interrupt::RxFifo0NewMsg);
            self.on_rx_irq();
        }
        if self.hw.has_interrupt(Interrupt::TxComplete) {
            self.hw.clear_interrupt(Interrupt::TxComplete);
            m_tx.lock(|tx| {
                if let Some(msg) = tx.queue.pop() {
                    tx.transmit(&msg);
                }
            });
        }
        if self.hw.has_interrupt(Interrupt::ErrPassive) {
            self.hw.clear_interrupt(Interrupt::ErrPassive);
            defmt::error!("CAN peripheral in Error Passive"); // TODO: how to recover?
        }
        if self.hw.has_interrupt(Interrupt::BusOff) {
            self.hw.clear_interrupt(Interrupt::BusOff);
            panic!("CAN peripheral in Bus Off");
        }
    }

    fn on_rx_irq(&mut self) {
        let frame = match self.hw_rx.receive_frame() {
            Ok(ReceiveOverrun::NoOverrun(frame)) => frame,
            Ok(ReceiveOverrun::Overrun(frame)) => {
                defmt::error!("CAN RX overrun reported");
                frame
            }
            // Shouldn't happen unless RX IRQ fired without anything received
            Err(err) => panic!("CAN internal error {err:?}"),
        };
        match self.rx_sender.try_send(frame) {
            Ok(_) => (),
            // Can probably leave this as panic for now, as app should be able to handle max incoming CAN rate
            Err(err) => panic!("CAN RX queue overflow {err:?}"),
        }
    }
}

// Public struct for a Queued CAN frame in either direction. Access as an embedded_can::Frame
// (FDCAN doesn't have a struct for this, it splits headers and their data up.)
#[derive(Clone, Debug)]
pub struct QueuedFrame {
    // TODO: make both these fields private once all access via embedded_can::Frame
    pub header: TxFrameHeader, // Note: TxFrameHeader is used for TX and RX directions
    pub data: [u8; 8],         // Fixed size array, see header.len for 'real' length
}

impl defmt::Format for QueuedFrame {
    fn format(&self, f: defmt::Formatter) {
        // format the bitfields of the register as struct fields
        defmt::write!(
            f,
            "CAN Frame (id={=u32:#x}, dlen={}, data={=[u8]:#04x})",
            match self.header.id.into() {
                Id::Standard(sid) => sid.as_raw().into(),
                Id::Extended(eid) => eid.as_raw(),
            },
            self.header.len,
            self.data[..self.header.len as usize],
        )
    }
}

impl Ord for QueuedFrame {
    fn cmp(&self, other: &Self) -> Ordering {
        FramePriority::from(self.header.id).cmp(&other.header.id.into())
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
        Self::new_tx(
            TxFrameHeader {
                len: data.len() as u8,
                frame_format: fdcan::frame::FrameFormat::Standard,
                id: Id::Standard(id).into(),
                bit_rate_switching: false,
                marker: None,
            },
            data,
        )
    }

    pub fn from_frame<F>(frame: F) -> Self where F: Frame {
        QueuedFrame::new(frame.id(), frame.data()).unwrap()
    }

    // Internal constructor for re-queue on transmit
    fn new_tx(header: TxFrameHeader, tx_data: &[u8]) -> Self {
        let mut data = [0_u8; 8];
        let dlen = min(header.len, 8) as usize;
        data[..dlen].copy_from_slice(&tx_data[..dlen]);
        Self { header, data }
    }

    // Function to pass to transmit_pending(), dequeues a pending
    // message as QueuedFrame
    fn from_pending_transmit(_: fdcan::Mailbox, header: TxFrameHeader, data32: &[u32]) -> Self {
        // Awkward workaround for fdcan passing the data here as &[u32]
        let mut data = [0_u8; 8];
        let dlen = min(min(header.len as usize, core::mem::size_of_val(data32)), 8);
        unsafe {
            data[..dlen].copy_from_slice(&data32.align_to::<u8>().1[..dlen]);
        }
        QueuedFrame { header, data }
    }
}

impl Frame for QueuedFrame
{
    fn new(id: impl Into<Id>, data: &[u8]) -> Option<Self> {
        let len = data.len();
        if len > 8 {
            return None
        };
        Some(Self::new_tx(
            TxFrameHeader {
                id: id.into().into(), // urgh!
                len: len as u8,
                // No FD-CAN features supported for embedded_can::Frame
                frame_format: fdcan::frame::FrameFormat::Standard,
                bit_rate_switching: false,
                marker: None,
            },
            data,
        ))
    }

    fn new_remote(_id: impl Into<Id>, _dlc: usize) -> Option<Self> {
        None // No remote frame support here
    }

    fn is_extended(&self) -> bool {
        match self.header.id.into() {
            Id::Standard(_) => false,
            Id::Extended(_) => true,
        }
    }

    fn is_remote_frame(&self) -> bool {
        false
    }

    fn id(&self) -> Id {
        self.header.id.into()
    }

    fn dlc(&self) -> usize {
        self.header.len as usize
    }

    fn data(&self) -> &[u8] {
        &self.data
    }
}

impl Eq for QueuedFrame {}

// Public struct for the Tx side. Unlike Rx this isn't an RTIC queue
// and doesn't block
pub struct Tx<I: fdcan::Instance> {
    can: fdcan::Tx<I, NormalOperationMode>,
    queue: BinaryHeap<QueuedFrame, Max, TX_CAPACITY>,
}

impl<I: fdcan::Instance> Tx<I> {
    fn new(can: fdcan::Tx<I, NormalOperationMode>) -> Self {
        Self {
            can,
            queue: BinaryHeap::new(),
        }
    }

    #[inline]
    pub fn transmit(&mut self, msg: &impl Frame) {
        // Convert to a QueuedFrame here, to minimise monomorphisation

        // Panic: TODO unsure what to do about unwrap here?
        self.transmit_frame(QueuedFrame::new(msg.id(), msg.data()).unwrap());
    }

    fn transmit_frame(&mut self, frame: QueuedFrame) {
        //defmt::trace!("CAN TX {:?}", msg); // TODO: fix log line
        let maybe_queue = match self.can.transmit_preserve_frame(
            &frame,
            &mut QueuedFrame::from_pending_transmit,
        ) {
            // Happy path, there was an empty hardware TX buffer
            Ok(None) => None,
            // Preserve the pending TX message that was replaced in hardware
            Ok(Some(dequeued)) => Some(dequeued),
            // Rather than blocking on fdcan, queue this message for later transmit
            Err(nb::Error::WouldBlock) => Some(frame),
            // TODO: Figure out what this means given error type is Infallible here...
            Err(nb::Error::Other(_)) => panic!("Unexpected CAN TX error"),
        };
        if let Some(to_queue) = maybe_queue {
            match self.queue.push(to_queue) {
                Ok(_) => (),
                _ => {
                    defmt::error!("TX queue overflow");
                    // Generally all the data we send is only useful if fresh, so
                    // clear the transmit queue if it seems like the bus is offline
                    self.queue.clear();
                    // Clear the hardware TX mailboxes as well
                    self.can.abort(Mailbox::_0);
                    self.can.abort(Mailbox::_1);
                    self.can.abort(Mailbox::_2);
                }
            }
        }
    }
}
