// this module gives the required base in order to implement a system of calling
// a function from one task and running it on the other, with a returned and awaitable value

#[derive(Clone)]
pub struct TaskInterface<T> {
    pub id: usize,
    pub data: T,
}

#[macro_export]
macro_rules! other_task_runner_setup {
    ($prefix:ident, $req_enum:ty, $res_enum:ty) => {
        use paste::paste;
        use crate::tools::other_task_runner::TaskInterface;
        use embassy_sync::{
            blocking_mutex::raw::CriticalSectionRawMutex,
            channel::Channel,
            watch::Watch
        };
        use core::cell::Cell;
        use embassy_sync::mutex::Mutex;

        paste! {
            static [<$prefix:upper _REQ_ID>]: Mutex<CriticalSectionRawMutex, Cell<usize>> =
                Mutex::new(Cell::new(0));
            static [<$prefix:upper _REQ_CHANNEL>]: Channel<CriticalSectionRawMutex, TaskInterface<$req_enum>, 128> =
                Channel::new();
            static [<$prefix:upper _RES_WATCH>]: Watch<CriticalSectionRawMutex, TaskInterface<$res_enum>, 128> = Watch::new();

            async fn [<$prefix:lower _get_req_id>]() -> usize {
                let req_id_cell = [<$prefix:upper _REQ_ID>].lock().await;
                let req_id = req_id_cell.get();
                req_id_cell.set(req_id + 1);
                req_id
            }

            // This sends the request and waits for the response
            #[macro_export]
            macro_rules! [<$prefix:lower _call_request>] {
                ($variant:tt, $data:expr) => {
                    (async || {
                        let req_id = [<$prefix:lower _get_req_id>]().await;
                        [<$prefix:upper _REQ_CHANNEL>]
                            .send(TaskInterface {
                                id: req_id,
                                data: $req_enum::$variant($data)
                            })
                            .await;
                        let mut recv = [<$prefix:upper _RES_WATCH>].receiver().unwrap();
                        loop {
                            let res = recv.changed().await;
                            if res.id == req_id {
                                match res.data.clone() {
                                    $res_enum::$variant(res_data) => {
                                        return res_data;
                                    }
                                    _ => {
                                        panic!("The request {:?} did not have the expected variant", req_id);
                                    }
                                }
                            }
                        }
                    })().await
                }
            }

            // this sends the request and doesn't wait for the response
            #[macro_export]
            macro_rules! [<$prefix:lower _send_request>] {
                ($variant:tt, $data:expr) => {
                    (async || {
                        if ![<$prefix:upper _REQ_CHANNEL>].is_full() {
                            [<$prefix:upper _REQ_CHANNEL>]
                                .send(TaskInterface {
                                    id: [<$prefix:lower _get_req_id>]().await,
                                    data: $req_enum::$variant($data)
                                }).await;
                        }
                    })().await
                }
            }

            macro_rules! [<$prefix:lower _request_handler>] {
                ($req_handler:tt) => {
                    let sender = [<$prefix:upper _RES_WATCH>].sender();

                    loop {
                        let request = [<$prefix:upper _REQ_CHANNEL>].receive().await;

                        let res: $res_enum = match request.data $req_handler;

                        sender.send(TaskInterface {
                            id: request.id,
                            data: res
                        });
                    }
                }
            }

        }
    };
}
