import numpy as np
from scipy.linalg import toeplitz
from scipy.signal import lfilter
from pyEMG.metrics import vaf_score, vaf_mv_score

class WienerFilter(object):
    """Wiener Filter regression.
    Input features and target signals are mean subtracted and normalized to unit standard deviation.
    After prediction, output signals are transformed back into their original scales.

    Parameters
    ----------

    num_feat : int
        Number of input features.
    num_pred : int
        Number of outputs.
    reg_lambda : float
        Regularization hyper-parameter
    num_lags : int
        Maximum length of filters (i.e. number of lags) used for regression.

    Attributes
    ----------

    H : array-like, shape = (num_feat*num_lags,num_pred)
        Coefficient matrix
    _input_mean : array-like, shape = (num_feat)
        Mean vector of input features.
    _input_sigma : array-like, shape = (num_feat)
        Standard deviation vector of input features.
    _output_lims : tuple, each element is array-like, shape = (num_pred,)
           Range of target signals.
    metrics : Goodness of fit on training data
        metrics.vaf : Variance accounted for, for each target signal
        metrics.vaf_mv : Multivariate variance accounted for
    """

    def __init__(self, num_feat, num_pred, reg_lambda=1e-4, num_lags=1):
        self.num_feat = num_feat
        self.num_pred = num_pred
        self.reg_lambda = reg_lambda
        self.num_lags=num_lags
        self.H = np.empty((num_feat*num_lags,num_pred), dtype=float)
        self.vaf = None
        self.vaf_mv = None

    def __str__(self):
        return "Wiener filter regressor"

    def __repr__(self):
        return "Wiener filter regessor"

    @property
    def total_io(self):
        return self.num_feat + self.num_pred

    def _center_input(self, x, mu=None):
        x = np.asarray(x)
        if mu is None: # If mean is not provided, estimate it (for training data)
            mu = np.mean(x, axis=0)
            self._input_mean = mu
        x = np.subtract(x,self._input_mean)
        return x

    def _standardize_input(self, x, sigma=None):
        if sigma is None:
            sigma = np.std(x, axis=0)
            self._input_sigma = sigma
        x = np.divide(x,self._input_sigma)
        return x

    def _center_output(self, x, mu=None):
        x = np.asarray(x)
        if mu is None: # If mean is not provided, estimate it (for training data)
            mu = np.mean(x, axis=0)
            self._output_mean = mu
        x = np.subtract(x,self._output_mean)
        return x

    def _standardize_output(self, x, sigma=None):
        if sigma is None:
            sigma = np.std(x, axis=0)
            self._output_sigma = sigma
        x = np.divide(x,self._output_sigma)
        return x

    def _recover_output(self, x):
        #x = np.asarray(x)
        x = np.multiply(x, self._output_sigma)
        x = np.add(x, self._output_mean)
        return x

    def _covf(self, x,M):
        n_sam, n_dim = np.shape(x)
        x = np.vstack((x, np.zeros((M,n_dim))))
        rows = np.arange(n_sam)
        R = np.zeros((n_dim**2,M), dtype=float)
        for jj in range(M):
            a = np.dot(np.transpose(x[rows,:]), x[rows+jj,:])
            R[:,jj] = (np.conj(a)/n_sam).reshape((n_dim**2), order = 'F')
        return R


    def fit(self, X, Y):
        X = np.asarray(X)
        Y = np.asarray(Y)
        assert X.shape[1] == self.num_feat
        assert Y.shape[1] == self.num_pred
        assert X.shape[0] == Y.shape[0]

        # Store output minimum and maximum values
        self._output_range = np.max(Y, axis = 0) - np.min(Y, axis = 0)
        self._output_max = np.max(Y, axis = 0)
        self._output_min = np.min(Y, axis = 0)


#        # Center and standardize inputs
#        X  = self._center_input(X)
#        X = self._standardize_input(X)
#        #  Center and standardize outputs
        Y = self._center_output(Y)
        Y = self._standardize_output(Y)

        numio = self.total_io
        R =  self._covf(np.hstack((X,Y)),self.num_lags)
        PHI = np.empty((2*self.num_lags-1,numio**2), dtype = float,  order='C')

        for ii in range(numio):
            for jj in range(numio):
                PHI[:,ii+jj*numio] = np.hstack((R[jj+ii*numio,np.arange(self.num_lags-1,0,-1)], R[ii+jj*numio,:]))

        Nxxr = np.arange(self.num_lags-1, 2*(self.num_lags-1)+1,1)
        Nxxc = np.arange(self.num_lags-1,-1,-1)
        Nxy = np.arange(self.num_lags-1, 2*(self.num_lags-1)+1)


        # Solve matrix equations to identify filters
        PX = np.empty((self.num_feat*self.num_lags,self.num_feat*self.num_lags), dtype=float, order='C')
        for ii in range(self.num_feat):
            for jj in range(self.num_feat):
                c_start = ii*self.num_lags
                c_end = (ii+1)*self.num_lags
                r_start = jj*self.num_lags
                r_end = (jj+1)*self.num_lags
                PX[r_start:r_end,c_start:c_end] = toeplitz(PHI[Nxxc,ii+(jj)*numio],PHI[Nxxr,ii+(jj)*numio])

        PXY = np.empty((self.num_feat*self.num_lags, self.num_pred), dtype=float, order='C')
        for ii in range(self.num_feat):
            for jj in range(self.num_feat,self.num_feat+self.num_pred,1):
                r_start = ii*self.num_lags
                r_end = (ii+1)*self.num_lags
                c_ind = jj-self.num_feat
                PXY[r_start:r_end,c_ind] = PHI[Nxy,ii+(jj)*numio]

        self.H = np.linalg.solve((PX + self.reg_lambda*np.identity(PX.shape[0])), PXY)


    def predict(self, X, online=False, normalizePrediction=False):
        """If batch is True, X.shape = (num_sam, num_feat), if not
        X.shape = (num_lags, num_feat)."""
        X = np.asarray(X)
        # If input array is single-dimensional, reshape it
        if X.ndim == 1:
            X = X.reshape((1,X.shape[0]))
        # Make sure input matrix has the correct number of features
        assert X.shape[1] == self.num_feat

        # Make sure the model has been fit
        #if np.allclose(self.H, np.empty_like(self.H)):
            #raise('Model has not been fit yet.')

#        # Center and standardize inputs
#        X  = self._center_input(X, self._input_mean)
#        X = self._standardize_input(X, self._input_sigma)

        if online is not True:
            num_samples = X.shape[0]
            Y = np.zeros((num_samples,self.num_pred))
            for ii in range(self.num_pred):
                for jj in range(self.num_feat):
                    coef = self.H[jj*self.num_lags:(jj+1)*self.num_lags,ii]
                    Y[:,ii] += lfilter(coef,1,X[:,jj], axis = -1)

            Y = Y[self.num_lags-1:,:]
        else:
            X_ud = np.flipud(X)
            Y = np.dot(X_ud.reshape(-1, order='F'), self.H)

        Y = self._recover_output(Y)

        if normalizePrediction == True:
            Y = (Y - self._output_min)/(self._output_max-self._output_min)
            # Threshold in range [0,1]
            Y[Y<0] = 0
            Y[Y>1] = 1

        return Y

    def evaluate(self,X,Y):
        pred_training_data = self.predict(X)
        self.vaf = vaf_score(Y[self.num_lags-1:,:], pred_training_data)
        self.vaf_mv = vaf_mv_score(Y[self.num_lags-1:,:], pred_training_data)
