import numpy as np
import warnings
  
class MovementCrossValidation(object):
    
    def __init__(self, n_reps, n_folds, n_trn, n_mov=None):
        self.n_folds = n_folds
        self.n_reps = n_reps
        self.n_trn = n_trn
        self.n_tst = n_reps - n_trn
        self.n_mov = n_mov
        self._assert_parameters()
        self.train_instances = None
        self.test_instances = None
        
    
    def _assert_parameters(self):
        """Assert cross-validation parameters."""
        if self.n_reps == self.n_folds * self.n_tst:
            self._is_ill_defined = False # OK
        elif self.n_reps < self.n_folds * self.n_tst:
            raise ValueError('Desired cross-validation configuration is impossible.')
        elif self.n_reps > self.n_folds * self.n_tst:
            warnings.warn('Cross-validation configuration is ill-defined. Picking one at random.')
            self._is_ill_defined = True
            
    def _segpoints(self, stimulus):
        """Find segmentation points."""
        stim_diff = np.diff(stimulus)
        changepoints = np.nonzero(stim_diff)[0]
        changepoints = np.hstack([0, changepoints, stimulus.size])
        changepoints_diff = np.diff(changepoints)
        segpoints = changepoints[:-1] + np.fix(changepoints_diff/2)
        segpoints = segpoints[0::2] # Sub-sample every 2 points
        return segpoints                
    
    def _downsample(self, x, offset=0):
        """Decrease sampling rate by integer factor."""
        x = np.asarray(x)
        if offset >= self.n_reps:
            offset = np.mod(offset, self.n_reps)
        ind = np.arange(start=offset, stop=x.size, step= self.n_reps)
        if offset == 0:
            ind = ind[0:-1]
        return x[ind]
        
    def fit(self, stimulus):
        stimulus = np.asarray(stimulus)
        if stimulus.ndim == 2:
            warnings.warn("Stimulus is 2-dimensional. Squeezing")
            stimulus = np.squeeze(stimulus)
        self.n_mov = np.unique(stimulus).size-1 if self.n_mov is None else self.n_mov
        segpoints = self._segpoints(stimulus)
        self.train_instances = []
        self.test_instances = []
        if self._is_ill_defined is True:
            raise NotImplementedError
        else:
            points = []
            for rep in range(self.n_reps):
                points.append(self._downsample(x=segpoints, offset=rep))
            points = np.asarray(points, dtype=int)
    
            for fold in range(0, self.n_folds):
                tst_ind = []
                for mvmnt in range(self.n_mov):
                    if (fold+1)*self.n_tst is not self.n_reps:
                        tst_ind.extend(range(points[fold*self.n_tst,mvmnt],points[(fold+1)*self.n_tst,mvmnt]))
                    else:
                        if mvmnt is not self.n_mov-1:
                            tst_ind.extend(range(points[fold*self.n_tst,mvmnt], points[0,mvmnt+1]))
                        else:
                            tst_ind.extend(range(points[fold*self.n_tst,mvmnt], stimulus.size))
                tst_ind = np.asarray(tst_ind)
                
                tr_ind = np.arange(stimulus.size)
                tr_ind = np.delete(tr_ind, tr_ind[tst_ind])
                self.train_instances.append(tr_ind)
                self.test_instances.append(tst_ind) 
    
    def get_data(self, data, fold):
        """Return training and testing data for a specified fold"""
        

        data_tr = data[self.train_instances[fold]]
        data_ts = data[self.test_instances[fold]]
        return (data_tr, data_ts)

                       
## TESTS
#points = []   
#train_instances=[]
#test_instances=[] 
#
#fold = 0
#plt.figure()
#plt.plot(stimulus)      
#plt.scatter(indices_testing[fold], 0.5*np.ones_like(indices_testing[fold]),color='red')
#plt.scatter(indices_training[fold],  0.5*np.ones_like(indices_training[fold]),color='green')
#                

    

        