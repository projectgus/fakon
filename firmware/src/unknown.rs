use fugit::RateExtU32;
use hex_literal::hex;
use rtic::Mutex;

use crate::{
    can_queue,
    can_utils::OutgoingMessage,
    car,
    dbc::pcan::{self, Unk5b3PowerState, Unk5b3UnkPowerRelated},
    hardware,
    periodic::PeriodicGroup,
};

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

    // On 2019 this is mostly constant except for one signal, but even that
    // signal only seems to count sometimes while moving... *shrug*
    let unk462 = pcan::Unk462::try_from(hex!("FE3FFF1FF01F0000").as_ref()).unwrap();

    let mut unk471 = pcan::Unk471::try_from(hex!("14001000000C").as_ref()).unwrap();

    let unk4fe = pcan::Unk4fe::try_from(hex!("FFFF7FFFFF00FFFF").as_ref()).unwrap();

    let mut unk50d = pcan::Unk50d::try_from(hex!("0000300150000000").as_ref()).unwrap();
    let mut unk50d_counter = 0;

    // Note: currently not modelling the "startup" pulse seen in logs
    // where one bit changes
    let unk520 = pcan::Unk520::try_from(hex!("0000000000000000").as_ref()).unwrap();

    // Mystery message with a single bit, often set when in Park...
    let unk559 = pcan::Unk559::new(false).unwrap();

    // This message is always zeroes in the 2019 logs, although changes to another
    // pattern on the 2021
    let unk55f = pcan::Unk55f::try_from(hex!("0000000000000000").as_ref()).unwrap();

    // First two bytes here are unique to my 2019 Kona, unsure if that matters
    let unk55c = pcan::Unk55c::try_from(hex!("071F14FF01000000").as_ref()).unwrap();

    // Another constant-looking message, although is sometimes all zeroes in logs (briefly)
    let unk561 = pcan::Unk561::try_from(hex!("0560000780000F00").as_ref()).unwrap();

    let unk578 = pcan::Unk578::try_from(hex!("000000000000").as_ref()).unwrap();

    
    let mut unk588 = pcan::Unk588::new(0x00, 0x00, 0x00).unwrap();

    // This message has values which change occasionally, but unclear what they signify. This is the most
    // common set of values...
    let unk593 = pcan::Unk593::new(0x24, 0x00, 0xFFFFFFFF).unwrap();

    let mut unk5b3 = pcan::Unk5b3::new(0xF, 0xFF, 0xFF, 0xF).unwrap();

    // Most common set of values in 2019 logs. 2021 has different values here,
    // and 2019 also sometimes has one bit different to this...
    let unk5ca = pcan::Unk5ca::try_from(hex!("000000FEFE500000").as_ref()).unwrap();

    let mut group = PeriodicGroup::new(100.Hz());
    let mut every_1hz = group.new_period(1.Hz());
    let mut every_5hz = group.new_period(5.Hz());
    let mut every_10hz = group.new_period(10.Hz());
    let mut every_50hz = group.new_period(50.Hz());

    loop {
        group.next_poll().await;

        let ignition = car.lock(|car| car.ignition());

        if ignition.ig3_on() && every_1hz.due(&group) {
            pcan_tx.lock(|tx| tx.transmit(&unk5ca));
        }

        if every_5hz.due(&group) {
            // Update 5b3 from ignition
            unk5b3.set_unk_power_related(if ignition.ig3_on() {
                Unk5b3UnkPowerRelated::PowerOn
            } else {
                Unk5b3UnkPowerRelated::PoweredOff
            }.into()).unwrap();
            if ignition != car::Ignition::Off {
                unk5b3.set_unknown2(0x10).unwrap();
            }
            unk5b3.set_power_state(match (ignition, unk5b3.power_state()) {
                (car::Ignition::Off, Unk5b3PowerState::Off) => Unk5b3PowerState::Off,
                (car::Ignition::Off, _) => Unk5b3PowerState::GoingToSleep,
                (car::Ignition::IG3, _) => Unk5b3PowerState::On,
                (car::Ignition::On, _) => Unk5b3PowerState::On,
            }.into()).unwrap();

            if ignition.ig3_on() {
                unk50d_counter += 1;
                if unk50d_counter >= 200 {
                    // Every 200 5Hz messages we increment the 10s counter
                    // (using wrapping_add even though this takes 7 days to wrap...)
                    unk50d
                        .set_counter10s(unk50d.counter10s().wrapping_add(1))
                        .unwrap();
                    unk50d_counter = 0;
                }
            } else {
                unk50d_counter = 0;
                unk50d.set_counter10s(0).unwrap();
            }

            pcan_tx.lock(|tx| {
                if ignition.ig3_on() {
                    tx.transmit(&zeroes412);
                    tx.transmit(&zeroes45c);
                    tx.transmit(&unk50d);
                    tx.transmit(&unk559);
                }
                tx.transmit(&zeroes45d);
                tx.transmit(&zeroes45e);
                tx.transmit(&unk4fe);
                tx.transmit(&unk5b3);
            });
        }

        if every_50hz.due(&group) {
            unk471.update();

            pcan_tx.lock(|tx| {
                tx.transmit(&unk450);
                tx.transmit(&unk462);
                tx.transmit(&unk471);
            });
        }

        if every_10hz.due(&group) {
            // Unk588 reflects vehicle status in some way, not exactly
            // clear how
            let (b0, b1, b2) = match ignition {
                car::Ignition::Off => (0x00, 0x00, 0x00),
                car::Ignition::IG3 => (0x58, 0x1c, 0x00),
                car::Ignition::On => (0xfc, 0xff, 0x03),
            };
            unk588.set_unk0(b0).unwrap();
            unk588.set_unk1(b1).unwrap();
            unk588.set_unk2(b2).unwrap();

            pcan_tx.lock(|tx| {
                tx.transmit(&unk520);
                tx.transmit(&unk55c);
                tx.transmit(&unk55f);
                tx.transmit(&unk561);
                tx.transmit(&unk578);
                tx.transmit(&unk588);
                tx.transmit(&unk593);
            });
        }
    }
}
