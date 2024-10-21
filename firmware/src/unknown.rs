use fugit::RateExtU32;
use hex_literal::hex;
use rtic::Mutex;

use crate::{can_queue, can_utils::OutgoingMessage, car, dbc::pcan, hardware, periodic::PeriodicGroup};

// CAN messages from an unknown source
//
// Some of these are *probably* IGPM gateway messages, but they might be from some
// other module (and/or not needed) and their function is mostly a mystery.

pub async fn task_unknown<MPCAN, MCAR>(mut pcan_tx: MPCAN, mut car: MCAR) -> !
where
    MPCAN: Mutex<T = can_queue::Tx<hardware::PCAN>>,
    MCAR: Mutex<T = car::CarState>,
{
    // Weird collection of messages that are always all-zero in the logs...?
    let zeroes412 = pcan::Unk412::try_from(hex!("0000000000000000").as_ref()).unwrap();
    let zeroes45c = pcan::Unk45c::try_from(hex!("0000000000000000").as_ref()).unwrap();
    // this messages is mostly all zeroes but in some logs it's FFFFFFFFFFFFFF0F instead...?
    let zeroes45d = pcan::Unk45d::try_from(hex!("0000000000000000").as_ref()).unwrap();
    // this one is either all zeroes or FFFFFFFFFF000000
    let zeroes45e = pcan::Unk45e::try_from(hex!("0000000000000000").as_ref()).unwrap();

    let unk450 = pcan::Unk450::new(0, 0x1804).unwrap(); // speed and maybe cruise control buttons?

    let mut unk471 = pcan::Unk471::try_from(hex!("14001000000C").as_ref()).unwrap();

    // Mystery message with a single bit, often set when in Park...
    let unk559 = pcan::Unk559::new(false).unwrap();

    // First two bytes here are unique to my 2019 Kona, unsure if that matters
    let unk55c = pcan::Unk55c::try_from(hex!("071F14FF01000000").as_ref()).unwrap();

    // Another constant-looking message, although is sometimes all zeroes in logs (briefly)
    let unk561 = pcan::Unk561::try_from(hex!("0560000780000F00").as_ref()).unwrap();

    // This message has values which change occasionally, but unclear what they signify. This is the most
    // common set of values...
    let unk593 = pcan::Unk593::new(0x24, 0x00, 0xFFFFFFFF).unwrap();

    let mut group = PeriodicGroup::new(100.Hz());
    let mut every_5hz = group.new_period(5.Hz());
    let mut every_10hz = group.new_period(10.Hz());
    let mut every_50hz = group.new_period(50.Hz());

    loop {
        group.next_poll().await;

        let ig3_on = car.lock(|car| car.ignition().ig3_on());

        if ig3_on && every_5hz.due(&group) {
            pcan_tx.lock(|tx| {
                tx.transmit(&zeroes412);
                tx.transmit(&zeroes45c);
                tx.transmit(&zeroes45d);
                tx.transmit(&zeroes45e);
                tx.transmit(&unk559);
            });
        }

        if ig3_on && every_50hz.due(&group) {
            unk471.update();

            pcan_tx.lock(|tx| {
                tx.transmit(&unk471);
                tx.transmit(&unk450)
            });
        }

        if every_10hz.due(&group) {

            pcan_tx.lock(|tx| {
                // Unclear if this message should be gated on ignition state
                tx.transmit(&unk55c);

                tx.transmit(&unk561);

                // This message is sent even when vehicle is Off
                tx.transmit(&unk593);
            });
        }
    }
}
