use num_traits::Float;

#[derive(Debug, Clone)]
pub struct LinearLut<F: Float, const N: usize> {
    pub points: [[F; 2]; N],
}

impl<F: Float, const N: usize> LinearLut<F, N> {
    pub fn get(&self, x: F) -> F {
        if self.points.is_empty() {
            return F::zero();
        }
        if x <= self.points[0][0] {
            return Self::linear_map(self.points[0], self.points[1], x);
        }
        if x >= self.points[self.points.len() - 1][0] {
            return Self::linear_map(
                self.points[self.points.len() - 2],
                self.points[self.points.len() - 1],
                x,
            );
        }
        for i in 0..self.points.len() - 1 {
            if x >= self.points[i][0] && x <= self.points[i + 1][0] {
                return Self::linear_map(self.points[i], self.points[i + 1], x);
            }
        }
        return F::zero();
    }

    fn linear_map(p1: [F; 2], p2: [F; 2], x: F) -> F {
        let x0 = p1[0];
        let y0 = p1[1];
        let x1 = p2[0];
        let y1 = p2[1];
        y0 + (y1 - y0) * (x - x0) / (x1 - x0)
    }
}
