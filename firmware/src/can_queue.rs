use bxcan::{self};
use heapless::binary_heap::Max;
use heapless::BinaryHeap;
use rtic_sync::{channel, make_channel};

// Module to support software TX and RX queued CAN on rtic
//
// TX side uses a binary heap to send out messages in priority order.
// RX side writes messages into an RTIC channel for processing by the app.

// CAN TX and RX software queue sizes
const RX_CAPACITY: usize = 8;
const TX_CAPACITY: usize = 16;

// Types for each end of the Rx rtic channel
pub type RxReceiver = channel::Receiver<'static, bxcan::Frame, RX_CAPACITY>;
type RxSender = channel::Sender<'static, bxcan::Frame, RX_CAPACITY>;

pub(crate) fn init<I>(can: I) -> (Rx<I>, RxReceiver, TxQueue<I>)
where
    I: bxcan::FilterOwner,
{
    // Setup CAN

    // APB1 (PCLK1): 36MHz, Bit rate: 500kBit/s, Sample Point 87.5%
    // Value was calculated with http://www.bittiming.can-wiki.info/
    let mut can = bxcan::Can::builder(can)
        .set_bit_timing(0x001e0003)
        .leave_disabled();

    can.modify_filters()
        .enable_bank(0, bxcan::Fifo::Fifo0, bxcan::filter::Mask32::accept_all());

    // Sync to the bus and start normal operation.
    can.enable_interrupts(
        bxcan::Interrupts::TRANSMIT_MAILBOX_EMPTY
            | bxcan::Interrupts::FIFO0_MESSAGE_PENDING
            | bxcan::Interrupts::ERROR,
    );
    nb::block!(can.enable_non_blocking()).unwrap();

    let (can_tx, can_rx, _) = can.split();

    let (rx, rx_receiver) = Rx::new(can_rx);

    (rx, rx_receiver, TxQueue::new(can_tx))
}

#[derive(Clone, Debug, Eq, PartialEq)]
struct QueuedFrame(pub bxcan::Frame);

impl PartialOrd for QueuedFrame {
    fn partial_cmp(&self, other: &Self) -> Option<core::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for QueuedFrame {
    fn cmp(&self, other: &Self) -> core::cmp::Ordering {
        self.0.priority().cmp(&other.0.priority())
    }
}

impl From<bxcan::Frame> for QueuedFrame {
    fn from(value: bxcan::Frame) -> Self {
        QueuedFrame(value)
    }
}

impl From<QueuedFrame> for bxcan::Frame {
    fn from(value: QueuedFrame) -> Self {
        value.0
    }
}

pub struct TxQueue<I: bxcan::Instance> {
    can: bxcan::Tx<I>,
    queue: BinaryHeap<QueuedFrame, Max, TX_CAPACITY>,
}

pub trait Tx {
    fn transmit(&mut self, msg: &bxcan::Frame);
}

impl<I: bxcan::Instance> TxQueue<I> {
    fn new(can: bxcan::Tx<I>) -> Self {
        Self {
            can,
            queue: BinaryHeap::new(),
        }
    }

    pub fn on_tx_irq(&mut self) {
        if let Some(msg) = self.queue.pop() {
            self.transmit(&msg.into());
        }
    }
}

impl<I: bxcan::Instance> Tx for TxQueue<I> {
    fn transmit(&mut self, msg: &bxcan::Frame) {
        let maybe_queue = match self.can.transmit(msg) {
            Ok(status) => status.dequeued_frame().cloned(),
            // Rather than blocking on bxcan, queue this message for later transmit
            Err(nb::Error::WouldBlock) => Some(msg.clone()),
            _ => None,
        };
        if let Some(to_queue) = maybe_queue {
            match self.queue.push(to_queue.clone().into()) {
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

pub struct Rx<I: bxcan::Instance> {
    can: bxcan::Rx0<I>,
    rx_sender: RxSender,
}

impl<I: bxcan::Instance> Rx<I> {
    fn new(can: bxcan::Rx0<I>) -> (Self, RxReceiver) {
        let (sender, receiver) = make_channel!(bxcan::Frame, RX_CAPACITY);
        (
            Self {
                can,
                rx_sender: sender,
            },
            receiver,
        )
    }

    pub fn on_rx_irq(&mut self) {
        let msg = match self.can.receive() {
            Ok(msg) => msg,
            Err(err) => panic!("CAN RX error {err:?}"), // TODO: handle CAN errors
        };
        match self.rx_sender.try_send(msg) {
            Ok(_) => (),
            // Can probably leave this as panic for now, as app should be able to handle max incoming CAN rate
            Err(err) => panic!("CAN RX overrun {err:?}"),
        }
    }
}

