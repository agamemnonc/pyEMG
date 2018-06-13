# -*- coding: utf-8 -*-
"""
Created on Sat Mar 19 12:18:59 2016

@author: Agamemnon
"""
import numpy as np
from scipy.stats.mstats import mode
from scipy.signal import lfilter
from scipy.fftpack import fft, ifft
from pyEMG.utils import nextpow2

def get_mav_feat(x):
    """Mean absolute value feature. """
    return np.mean(np.abs(x), axis=0)

def get_mv_feat(x):
    """Mean value feature. """
    return np.mean(x, axis=0)

def get_var_feat(x):
    """Log-variance feature. """
    return np.var(x, axis = 0)

def get_logvar_feat(x):
    """Log-variance feature. """
    return np.log(np.var(x, axis = 0))

def get_wamp_feat(x,threshold=5e-6):
    """Wilson amplitude feature. """
    return np.sum(np.diff(x, n=1, axis=0) < -threshold, axis=0)

def get_wl_feat(x):
    """Waveform length feature. """
    return np.sum(np.absolute(np.diff(x, n=1, axis=0)), axis=0)

def get_ssc_feat(x, deadzone = 4.5e-6):
    """Slope sign change feature. """
    y = np.vstack((np.zeros((1,x.shape[1])), np.diff(x, axis = 0)))
    y = (y > deadzone).astype(int) - (y < - deadzone).astype(int)
    a = 1
    b = np.exp(-(np.arange(1,(x.shape[0]/2.)+1)))
    z = lfilter(b,a,y)
    z = (z > 0).astype(int) - (z < 0).astype(int)
    dz = np.diff(z, axis = 0)
    return np.sum(np.abs(dz)==2, axis = 0)

def get_ar_feat(x,order=4):
    x_lpc = np.real(_lpc(x,order))[1:].T
    return -x_lpc.reshape(1,-1)

def get_quantile_feat(x, q = [0.1, 0.25, 0.5, 0.75, 0.9]):
    """Quantile feature."""
    q = 100*np.asarray(q)
    return np.percentile(x, q, axis=0).T.reshape(1,-1)

def get_int_mode_feat(x):
    """Mode integer value feature. """
    return np.asarray(mode(x).mode, dtype=int)



def _levinson(r, order=None, allow_singularity=False):
    r"""Levinson-Durbin recursion.

    Find the coefficients of a length(r)-1 order autoregressive linear process

    :param r: autocorrelation sequence of length N + 1 (first element being the zero-lag autocorrelation)
    :param order: requested order of the autoregressive coefficients. default is N.
    :param allow_singularity: false by default. Other implementations may be True (e.g., octave)

    :return:
        * the `N+1` autoregressive coefficients :math:`A=(1, a_1...a_N)`
        * the prediction errors
        * the `N` reflections coefficients values

    This algorithm solves the set of complex linear simultaneous equations
    using Levinson algorithm.

    """
    T0  = np.real(r[0])
    T = r[1:]
    M = len(T)

    if order == None:
        M = len(T)
    else:
        assert order <= M, 'order must be less than size of the input data'
        M = order

    realdata = np.isrealobj(r)
    if realdata is True:
        A = np.zeros(M, dtype=float)
        ref = np.zeros(M, dtype=float)
    else:
        A = np.zeros(M, dtype=complex)
        ref = np.zeros(M, dtype=complex)

    P = T0

    for k in range(0, M):
        save = T[k]
        if k == 0:
            temp = -save / P
        else:
            #save += sum([A[j]*T[k-j-1] for j in range(0,k)])
            for j in range(0, k):
                save = save + A[j] * T[k-j-1]
            temp = -save / P
        if realdata:
            P = P * (1. - temp**2.)
        else:
            P = P * (1. - (temp.real**2+temp.imag**2))
        if P <= 0 and allow_singularity==False:
            raise ValueError("singular matrix")
        A[k] = temp
        ref[k] = temp # save reflection coeff at each step
        if k == 0:
            continue

        khalf = (k+1)/2
        if realdata is True:
            for j in range(0, khalf):
                kj = k-j-1
                save = A[j]
                A[j] = save + temp * A[kj]
                if j != kj:
                    A[kj] += temp*save
        else:
            for j in range(0, khalf):
                kj = k-j-1
                save = A[j]
                A[j] = save + temp * A[kj].conjugate()
                if j != kj:
                    A[kj] = A[kj] + temp * save.conjugate()

    return A, P, ref

def _lpc(x,order):
    """Linear predictor coefficients. Supports 1D and 2D arrays only through
    iteration."""

    x = np.asarray(x)
    if x.ndim == 1:
        m = x.size
        X = fft(x,n=nextpow2(m))  # TODO nextpower of 2
        R = np.real(ifft(np.abs(X)**2)) # Auto-correlation matrix
        R = R/m
        a = _levinson(r=R, order=order)[0]
        a = np.hstack((1., a))
    elif x.ndim == 2:
        m,n = x.shape

        X = fft(x,n=nextpow2(m), axis=0)
        R = np.real(ifft(np.abs(X)**2, axis=0)) # Auto-correlation matrix
        R = R/m
        a = np.ones((order+1,n))
        for col in range(n):
            a[1:,col] = _levinson(r=R[:,col], order=order)[0]
    else:
        raise ValueError('Supported for 1-D or 2-D arrays only.')

    return a
