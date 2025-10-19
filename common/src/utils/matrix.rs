use nalgebra::{allocator::Allocator, Const, RawStorage, RealField, SMatrix, Scalar, Storage};

#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Mat<T, const R: usize, const C: usize> {
    data: [[T; R]; C],
}

pub type Vec<T, const R: usize> = Mat<T, R, 1>;

pub type Mat2<T> = Mat<T, 2, 2>;
pub type Mat3<T> = Mat<T, 3, 3>;
pub type Mat4<T> = Mat<T, 4, 4>;
pub type Mat5<T> = Mat<T, 5, 5>;
pub type Mat6<T> = Mat<T, 6, 6>;

pub type Vec2<T> = Vec<T, 2>;
pub type Vec3<T> = Vec<T, 3>;
pub type Vec4<T> = Vec<T, 4>;
pub type Vec5<T> = Vec<T, 5>;
pub type Vec6<T> = Vec<T, 6>;

macro_rules! impl_vec {
    ($type:ident: $($arg:ident),+) => {
        impl <T> $type<T> {
            pub fn new($($arg: T),+) -> Self {
                Self { data: [[$($arg),+]] }
            }
        }
    };
}

impl_vec!(Vec2: v1, v2);
impl_vec!(Vec3: v1, v2, v3);
impl_vec!(Vec4: v1, v2, v3, v4);
impl_vec!(Vec5: v1, v2, v3, v4, v5);
impl_vec!(Vec6: v1, v2, v3, v4, v5, v6);

pub type RMatrix<'a, T, const R: usize, const C: usize> =
    nalgebra::Matrix<T, Const<R>, Const<C>, RefStorage<'a, T, R, C>>;

impl<'a, T: Scalar, const R: usize, const C: usize> Into<RMatrix<'a, T, R, C>>
    for &'a Mat<T, R, C>
{
    fn into(self) -> RMatrix<'a, T, R, C> {
        RMatrix::from_data(RefStorage { data: &self.data })
    }
}

pub trait RefMatrix<T, const R: usize, const C: usize> {
    fn ref_matrix<'a>(&'a self) -> RMatrix<'a, T, R, C>;
}

impl<T: Scalar, const R: usize, const C: usize> RefMatrix<T, R, C> for &SMatrix<T, R, C> {
    fn ref_matrix<'a>(&'a self) -> RMatrix<'a, T, R, C> {
        RMatrix::from_data(RefStorage { data: &self.data.0 })
    }
}

impl<T: Scalar, const R: usize, const C: usize> RefMatrix<T, R, C> for &Mat<T, R, C> {
    fn ref_matrix<'a>(&'a self) -> RMatrix<'a, T, R, C> {
        RMatrix::from_data(RefStorage { data: &self.data })
    }
}

impl<T: Scalar, const R: usize, const C: usize> Mat<T, R, C> {
    pub fn nalg(&self) -> RMatrix<'_, T, R, C> {
        self.into()
    }
}

impl<T, const R: usize, const C: usize> From<nalgebra::SMatrix<T, R, C>> for Mat<T, R, C> {
    fn from(value: nalgebra::SMatrix<T, R, C>) -> Self {
        Self { data: value.data.0 }
    }
}

impl<T: RealField, const R: usize, const C: usize, const X: usize> core::ops::Mul<Mat<T, C, X>>
    for Mat<T, R, C>
{
    type Output = Mat<T, R, X>;

    fn mul(self, rhs: Mat<T, C, X>) -> Self::Output {
        (self.nalg() * rhs.nalg()).into()
    }
}

pub struct RefStorage<'a, T, const R: usize, const C: usize> {
    pub data: &'a [[T; R]; C],
}

unsafe impl<T: Scalar, const R: usize, const C: usize> RawStorage<T, Const<R>, Const<C>>
    for RefStorage<'_, T, R, C>
{
    type RStride = Const<1>;
    type CStride = Const<R>;

    fn ptr(&self) -> *const T {
        core::ptr::addr_of!(self.data[0][0])
    }

    fn shape(&self) -> (Const<R>, Const<C>) {
        (Const::<R>, Const::<C>)
    }

    fn strides(&self) -> (Self::RStride, Self::CStride) {
        (Const::<1>, Const::<R>)
    }

    fn is_contiguous(&self) -> bool {
        true
    }

    unsafe fn as_slice_unchecked(&self) -> &[T] {
        self.data.as_flattened()
    }
}

unsafe impl<T: Scalar, const R: usize, const C: usize> Storage<T, Const<R>, Const<C>>
    for RefStorage<'_, T, R, C>
{
    fn into_owned(self) -> nalgebra::Owned<T, Const<R>, Const<C>>
    where
        nalgebra::DefaultAllocator: nalgebra::allocator::Allocator<Const<R>, Const<C>>,
    {
        Self::clone_owned(&self)
    }

    fn clone_owned(&self) -> nalgebra::Owned<T, Const<R>, Const<C>>
    where
        nalgebra::DefaultAllocator: nalgebra::allocator::Allocator<Const<R>, Const<C>>,
    {
        nalgebra::DefaultAllocator::allocate_from_iterator(
            Const::<R>,
            Const::<C>,
            self.data.as_flattened().iter().cloned(),
        )
    }

    fn forget_elements(self) {
        // Immutable references are copy, so no special action needed
    }
}
