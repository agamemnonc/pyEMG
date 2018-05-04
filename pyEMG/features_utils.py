"""Miscellaneous utilities for time series analysis.

"""
import numpy as np
from scipy import fftpack
from scipy.signal import signaltools

#-----------------------------------------------------------------------------
# Correlation/Covariance utils
#-----------------------------------------------------------------------------

def remove_bias(x, axis):
    "Subtracts an estimate of the mean from signal x at axis"
    padded_slice = [slice(d) for d in x.shape]
    padded_slice[axis] = np.newaxis
    mn = np.mean(x, axis=axis)
    return x - mn[tuple(padded_slice)]


def crosscov(x, y, axis=-1, all_lags=False, debias=True, normalize=True):
    """Returns the crosscovariance sequence between two ndarrays.
    This is performed by calling fftconvolve on x, y[::-1]

    Parameters
    ----------

    x : ndarray
    y : ndarray
    axis : time axis
    all_lags : {True/False}
       whether to return all nonzero lags, or to clip the length of s_xy
       to be the length of x and y. If False, then the zero lag covariance
       is at index 0. Otherwise, it is found at (len(x) + len(y) - 1)/2
    debias : {True/False}
       Always removes an estimate of the mean along the axis, unless
       told not to (eg X and Y are known zero-mean)

    Returns
    -------

    cxy : ndarray
       The crosscovariance function

    Notes
    -----

    cross covariance of processes x and y is defined as

    .. math::

    C_{xy}[k]=E\{(X(n+k)-E\{X\})(Y(n)-E\{Y\})^{*}\}

    where X and Y are discrete, stationary (or ergodic) random processes

    Also note that this routine is the workhorse for all auto/cross/cov/corr
    functions.

    """
    if x.shape[axis] != y.shape[axis]:
        raise ValueError(
            'crosscov() only works on same-length sequences for now'
            )
    if debias:
        x = remove_bias(x, axis)
        y = remove_bias(y, axis)
    slicing = [slice(d) for d in x.shape]
    slicing[axis] = slice(None, None, -1)
    cxy = fftconvolve(x, y[tuple(slicing)].conj(), axis=axis, mode='full')
    N = x.shape[axis]
    if normalize:
        cxy /= N
    if all_lags:
        return cxy
    slicing[axis] = slice(N - 1, 2 * N - 1)
    return cxy[tuple(slicing)]


def crosscorr(x, y, **kwargs):
    """
    Returns the crosscorrelation sequence between two ndarrays.
    This is performed by calling fftconvolve on x, y[::-1]

    Parameters
    ----------

    x : ndarray
    y : ndarray
    axis : time axis
    all_lags : {True/False}
       whether to return all nonzero lags, or to clip the length of r_xy
       to be the length of x and y. If False, then the zero lag correlation
       is at index 0. Otherwise, it is found at (len(x) + len(y) - 1)/2

    Returns
    -------

    rxy : ndarray
       The crosscorrelation function

    Notes
    -----

    cross correlation is defined as

    .. math::

    R_{xy}[k]=E\{X[n+k]Y^{*}[n]\}

    where X and Y are discrete, stationary (ergodic) random processes
    """
    # just make the same computation as the crosscovariance,
    # but without subtracting the mean
    kwargs['debias'] = False
    rxy = crosscov(x, y, **kwargs)
    return rxy


def autocov(x, **kwargs):
    """Returns the autocovariance of signal s at all lags.

    Parameters
    ----------

    x : ndarray
    axis : time axis
    all_lags : {True/False}
       whether to return all nonzero lags, or to clip the length of r_xy
       to be the length of x and y. If False, then the zero lag correlation
       is at index 0. Otherwise, it is found at (len(x) + len(y) - 1)/2

    Returns
    -------

    cxx : ndarray
       The autocovariance function

    Notes
    -----

    Adheres to the definition

    .. math::

    C_{xx}[k]=E\{(X[n+k]-E\{X\})(X[n]-E\{X\})^{*}\}

    where X is a discrete, stationary (ergodic) random process
    """
    # only remove the mean once, if needed
    debias = kwargs.pop('debias', True)
    axis = kwargs.get('axis', -1)
    if debias:
        x = remove_bias(x, axis)
    kwargs['debias'] = False
    return crosscov(x, x, **kwargs)


def autocorr(x, **kwargs):
    """Returns the autocorrelation of signal s at all lags.

    Parameters
    ----------

    x : ndarray
    axis : time axis
    all_lags : {True/False}
       whether to return all nonzero lags, or to clip the length of r_xy
       to be the length of x and y. If False, then the zero lag correlation
       is at index 0. Otherwise, it is found at (len(x) + len(y) - 1)/2

    Notes
    -----

    Adheres to the definition

    .. math::

    R_{xx}[k]=E\{X[n+k]X^{*}[n]\}

    where X is a discrete, stationary (ergodic) random process

    """
    # do same computation as autocovariance,
    # but without subtracting the mean
    kwargs['debias'] = False
    return autocov(x, **kwargs)


def fftconvolve(in1, in2, mode="full", axis=None):
    """ Convolve two N-dimensional arrays using FFT. See convolve.

    This is a fix of scipy.signal.fftconvolve, adding an axis argument and
    importing locally the stuff only needed for this function

    """
    s1 = np.array(in1.shape)
    s2 = np.array(in2.shape)
    complex_result = (np.issubdtype(in1.dtype, np.complex) or
                      np.issubdtype(in2.dtype, np.complex))

    if axis is None:
        size = s1 + s2 - 1
        fslice = tuple([slice(0, int(sz)) for sz in size])
    else:
        equal_shapes = s1 == s2
        # allow equal_shapes[axis] to be False
        equal_shapes[axis] = True
        assert equal_shapes.all(), 'Shape mismatch on non-convolving axes'
        size = s1[axis] + s2[axis] - 1
        fslice = [slice(l) for l in s1]
        fslice[axis] = slice(0, int(size))
        fslice = tuple(fslice)

    # Always use 2**n-sized FFT
    fsize = 2 ** int(np.ceil(np.log2(size)))
    if axis is None:
        IN1 = fftpack.fftn(in1, fsize)
        IN1 *= fftpack.fftn(in2, fsize)
        ret = fftpack.ifftn(IN1)[fslice].copy()
    else:
        IN1 = fftpack.fft(in1, fsize, axis=axis)
        IN1 *= fftpack.fft(in2, fsize, axis=axis)
        ret = fftpack.ifft(IN1, axis=axis)[fslice].copy()
    del IN1
    if not complex_result:
        ret = ret.real
    if mode == "full":
        return ret
    elif mode == "same":
        if np.product(s1, axis=0) > np.product(s2, axis=0):
            osize = s1
        else:
            osize = s2
        return signaltools._centered(ret, osize)
    elif mode == "valid":
        return signaltools._centered(ret, abs(s2 - s1) + 1)


