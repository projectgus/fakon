// "CAN Gateway" messages ("Integrated Gateway Power Module").
// Some of these may originate from modules on other buses and
// be relayed via the IGPM, some generated by IGPM.
use crate::periodic::PeriodicGroup;
use crate::car;
use crate::can_queue::{self, QueuedFrame};
use crate::hardware::{self, Mono};
use fugit::RateExtU32;
use hex_literal::hex;
use rtic::Mutex;
use rtic_monotonics::Monotonic;

pub async fn task_igpm<MPCAN, MCAR>(mut pcan_tx: MPCAN, mut car: MCAR) -> !
where
    MPCAN: Mutex<T = can_queue::Tx<hardware::PCAN>>,
    MCAR: Mutex<T = car::CarState>,
{
    // first byte changes briefly during charge session
    let igpm_414 = QueuedFrame::new_std(0x414, &hex!("0000000000000000"));

    let mut igpm_541 = QueuedFrame::new_std(0x541, &[
        0x00,
        0x00,
        // drivers door closed, seatbelt on
        // needs at least one of these to go into D
        0x44,
        0x00,
        0x08,
        0x08, // "drive type option" bit set?
        0x00,
        0x00 ]);

    let mut igpm_clock = QueuedFrame::new_std(0x567, &hex!("0200000000000000"));

    let mut igpm_5ec = QueuedFrame::new_std(0x5ec, &[ 0; 8 ]);

    // Value changes one time in charge lot
    let igpm_5df = QueuedFrame::new_std(0x5df, &hex!("C5FFFF0100000000"));

    let igpm_553 = QueuedFrame::new_std(0x553, &hex!("0400000000008000"));

    // Odometer reading
    let igpm_5d0 = QueuedFrame::new_std(0x5d0, &hex!("9D13040000000000"));
    // HU_DATC_PE_00 (???)
    let igpm_5d3 = QueuedFrame::new_std(0x5d3, &hex!("0F00000000000000"));

    let mut group = PeriodicGroup::new(10.Hz());
    let mut period_10hz = group.new_period(10.Hz());
    let mut period_5hz = group.new_period(10.Hz());
    let mut period_1hz = group.new_period(1.Hz());

    loop {
        group.next_poll().await;

        // Car state is a small struct, cloning it means whole iteration
        // sees consistent state without holding the lock
        let car_state = car.lock(|c| c.clone());

        if period_10hz.due(&group) {
            // IGPM 541
            {
                if car_state.ignition_on {
                    igpm_541.data[0] = 0x03;
                    igpm_541.data[7] = 0x0C;  // note : parking brake switch also in this byte
                } else {
                    igpm_541.data[0] = 0x00;
                    igpm_541.data[7]= 0x00;
                }
            }

            // Clock
            {
                let secs = Mono::now().duration_since_epoch().to_secs();
                igpm_clock.data[3] = (secs % 60) as u8; // Seconds
                igpm_clock.data[2] = ((secs / 60) % 60) as u8; // Minutes
                igpm_clock.data[1] = ((secs / 60 / 60) % 24) as u8; // Hours
                igpm_clock.data[4] = 1; // "Valid" flag
            }

            // IGPM 5EC
            {
                // Is charge port locked?
                igpm_5ec.data[0] = if car_state.charge_port_locked { 1 } else { 0 };
            }

            pcan_tx.lock(|tx| {
                for m in [&igpm_414, &igpm_541, &igpm_clock, &igpm_5ec] {
                    tx.transmit(m)
                }
            });
        }

        if period_5hz.due(&group) {
            pcan_tx.lock(|tx| {
                for m in [&igpm_5df, &igpm_553] {
                    tx.transmit(m)
                }
            });
        }

        if period_1hz.due(&group) {
            pcan_tx.lock(|tx| {
                for m in [&igpm_5d0, &igpm_5d3] {
                    tx.transmit(m)
                }
            });
        }
    }
}