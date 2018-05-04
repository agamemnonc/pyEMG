import numpy as np
from bin_parm import BinParm
from datasets import Dataset, DatasetRaw, DatasetBinned

class Features(object):

    def __init__(self, sRate, binparm):
        self._win_size =  binparm.winsize*1e-3*sRate
        self._win_inc =  binparm.wininc*1e-3*sRate

class EmgFeatures(Features):

    def __init__(self, x, sRate, binparm):
        x = np.asarray(x)
        self._win_size =  binparm.winsize*1e-3*sRate
        self._win_inc =  binparm.wininc*1e-3*sRate
        wl = self._get_wl_feat(x, win_size=self._win_size, win_inc=self._win_inc)
        wamp = self._get_wamp_feat(x, win_size=self._win_size, win_inc=self._win_inc,threshold=5e-6)
        self.features = np.hstack((wl, wamp))


    def _get_wamp_feat(self, x, win_size, win_inc, threshold):
        num_sam, num_dim = np.shape(x)
        num_win = int(np.floor((num_sam-win_size)/win_inc))+1
        y = np.zeros((num_win,num_dim))
        st = 0
        en = win_size-1
        for ii in range(num_win):
            curwin = x[st:en,:]
            y[ii,:] = np.sum(np.diff(curwin,n=1, axis=0) > threshold, axis=0) + \
                            np.sum(np.diff(curwin,n=1, axis=0) < -threshold, axis=0)
            st += win_inc
            en += win_inc
        return y


    def _get_wl_feat(self, x, win_size, win_inc):
        num_sam, num_dim = np.shape(x)
        num_win = int(np.floor((num_sam-win_size)/win_inc))+1
        y = np.zeros((num_win,num_dim))
        st = 0
        en = win_size-1
        for ii in range(num_win):
            curwin = x[st:en,:]
            y[ii,:] = np.sum(np.absolute(np.diff(curwin,n=1, axis=0)), axis=0)
            st += win_inc
            en += win_inc
        return y


class AccFeatures(Features):

    def __init__(self,x, sRate, binparm):
        x = np.asarray(x)
        self._win_size =  binparm.winsize*1e-3*sRate
        self._win_inc =  binparm.wininc*1e-3*sRate
        mv = self._get_mv_feat(x, win_size=self._win_size, win_inc=self._win_inc)
        self.features = mv

    def _get_mv_feat(self, x, win_size, win_inc):
        num_sam, num_dim = np.shape(x)
        num_win = int(np.floor((num_sam-win_size)/win_inc))+1
        y = np.zeros((num_win,num_dim))
        st = 0
        en = win_size-1
        for ii in range(num_win):
            curwin = x[st:en,:]
            y[ii,:] = np.mean(curwin, axis = 0)
            st += win_inc
            en += win_inc
        return y

def combine_emg_acc_features(emgfeat, accfeat, sRate, binparm):

    assert(emgfeat._win_size==binparm.winsize*1e-3*sRate)
    assert(accfeat._win_size==binparm.winsize*1e-3*sRate)
    assert(emgfeat._win_inc==binparm.wininc*1e-3*sRate)
    assert(accfeat._win_inc==binparm.wininc*1e-3*sRate)

    feat = Features(sRate,binparm)
    feat.features = np.hstack((emgfeat.features, accfeat.features))
    return feat
