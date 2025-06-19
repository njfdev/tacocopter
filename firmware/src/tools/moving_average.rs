pub struct F32MovingAverage<const N: usize> {
    samples: [f32; N],
    length: usize,
    position: usize,
}

impl<const N: usize> F32MovingAverage<N> {
    pub fn new() -> Self {
        Self {
            samples: [0.0; N],
            length: 0,
            position: 0,
        }
    }

    pub fn get_average(&self) -> f32 {
        if self.length == 0 {
            return 0.0;
        }

        let mut total = 0.0;
        for i in 0..self.length {
            total += self.samples[i];
        }
        total / (self.length as f32)
    }

    pub fn sample(&mut self, sample: f32) {
        self.position += 1;
        if self.position >= N {
            self.position = 0;
        }
        if self.length != N {
            self.length += 1;
        }
        self.samples[self.position] = sample;
    }

    pub fn reset(&mut self) {
        self.samples = [0.0; N];
        self.length = 0;
        self.position = 0;
    }
}
