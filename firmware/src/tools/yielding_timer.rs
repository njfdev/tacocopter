use embassy_futures::yield_now;
use embassy_time::Instant;

pub struct YieldingTimer {}

/*
Instead of descheduling the task with normal Embassy Timer functions,
these functions keep the task alive and yield time to other tasks. It
also returns the "ending time" for use in loops (it accounts for going
over the desired time).
*/
impl YieldingTimer {
    pub async fn after_micros(time: u64) -> Instant {
        let start = Instant::now();
        while start.elapsed().as_micros() < time {
            yield_now().await;
        }
        Instant::from_micros(start.as_micros() + time)
    }
    pub async fn after_millis(time: u64) -> Instant {
        let start = Instant::now();
        while start.elapsed().as_millis() < time {
            yield_now().await;
        }
        Instant::from_micros(start.as_millis() + time)
    }
    pub async fn after_secs(time: u64) -> Instant {
        let start = Instant::now();
        while start.elapsed().as_secs() < time {
            yield_now().await;
        }
        Instant::from_micros(start.as_secs() + time)
    }
}
