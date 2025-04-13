use alloc::vec::Vec;
use core::ops::{Index, IndexMut};

#[derive(Debug)]
pub struct ArmMatrixF32 {
    data: Vec<f32>,
    rows: usize,
    cols: usize,
}

impl ArmMatrixF32 {
    pub fn new(rows: usize, cols: usize) -> Self {
        Self {
            rows,
            cols,
            data: {
                let mut v = Vec::with_capacity(rows * cols);
                v.resize(rows * cols, 0.0);
                v
            },
        }
    }

    pub fn as_slice(&self) -> &[f32] {
        self.data.as_slice()
    }

    pub fn as_mut_slice(&mut self) -> &mut [f32] {
        self.data.as_mut_slice()
    }

    pub fn row_count(&self) -> usize {
        self.rows
    }

    pub fn column_count(&self) -> usize {
        self.cols
    }
}

impl IndexMut<usize> for ArmMatrixF32 {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.data[index * self.cols..(index + 1) * self.cols]
    }
}

impl Index<usize> for ArmMatrixF32 {
    type Output = [f32];
    fn index(&self, index: usize) -> &Self::Output {
        &self.data[index * self.cols..(index + 1) * self.cols]
    }
}

pub fn mat_mult(m_a: &ArmMatrixF32, m_b: &ArmMatrixF32) -> ArmMatrixF32 {
    let mut m_c = ArmMatrixF32::new(m_a.rows, m_b.cols);

    let data_a = m_a.data.as_slice();
    let data_b = m_b.data.as_slice();
    let data_c = m_c.data.as_mut_slice();

    for i in 0..m_a.rows {
        for j in 0..m_b.cols {
            for k in 0..m_a.cols {
                data_c[i * m_b.cols + j] += data_a[i * m_a.cols + k] * data_b[k * m_b.cols + j];
            }
        }
    }
    m_c
}

pub fn mat_trans(m_a: &ArmMatrixF32) -> ArmMatrixF32 {
    let mut m_c = ArmMatrixF32::new(m_a.cols, m_a.rows);
    let data_a = m_a.data.as_slice();
    let data_c = m_c.data.as_mut_slice();
    for i in 0..m_a.rows {
        for j in 0..m_a.cols {
            data_c[j * m_a.rows + i] = data_a[i * m_a.cols + j];
        }
    }
    m_c
}
