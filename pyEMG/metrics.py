import numpy as np
from sklearn.metrics import accuracy_score, log_loss

def _check_x_y(true,pred):
    if true.shape != pred.shape:
        raise ValueError('True and predicted signals must be of the same size.')
    
def vaf_score(true,pred):
    true = np.asarray(true)
    pred = np.asarray(pred)
    _check_x_y(true,pred)

    # If input arrays are single-dimensional, reshape them
    if true.ndim == 1:
        true = true.reshape((1,true.shape[0]))      
    if pred.ndim == 1:
        pred = pred.reshape((1,pred.shape[0]))  
    
    n_sam, n_pred = true.shape
    ss_err = np.zeros(n_pred)
    ss_tot = np.zeros(n_pred)
    vaf = np.zeros(n_pred)
    for jj in xrange(n_pred):
        ss_err[jj] = np.sum((true[:,jj]-pred[:,jj])**2)
        ss_tot[jj] = np.sum((true[:,jj]-np.mean(true[:,jj]))**2)
        vaf[jj] = 1 - (ss_err[jj]/ss_tot[jj])

    return vaf
    

def vaf_mv_score(true, pred):
    true = np.asarray(true)
    pred = np.asarray(pred)
    _check_x_y(true,pred)

    # If input arrays are single-dimensional, reshape them
    if true.ndim == 1:
        true = true.reshape((1,true.shape[0]))      
    if pred.ndim == 1:
        pred = pred.reshape((1,pred.shape[0]))  
    
    n_sam, n_pred = true.shape
    ss_err = np.zeros(n_pred)
    ss_tot = np.zeros(n_pred)
    for jj in xrange(n_pred):
        ss_err[jj] = np.sum((true[:,jj]-pred[:,jj])**2)
        ss_tot[jj] = np.sum((true[:,jj]-np.mean(true[:,jj]))**2)
    
    vaf_mv = 1 - np.sum(ss_err)/np.sum(ss_tot)
    return vaf_mv

def balanced_accuracy_score(y_true, y_pred, method = 'edges', random_state=None):
    """Balanced classification accuracy metric (multi-class).
    Keeps only a subset of the data instances corresponding to the rest class.
    The size of the subset is equal to the median group size of the other 
    classes."""
    
    _check_x_y(y_true,y_pred)
    classes, n_instances = np.unique(y_true, return_counts=True)
    median_instances = np.median(n_instances[1:])
    n_classes = classes.size
    
    idx_rest = np.where(y_true == 0)[0] # Find rest instances
    idx_else = np.where(y_true != 0)[0] # Find all other instances
    
    if method == 'random':
        if random_state is not None:
            np.random.seed(random_state)
        idx_keep = np.random.choice(idx_rest,median_instances, replace=False) # Keep a random subset
        idx_final = np.sort(np.hstack((idx_keep, idx_else)))

    
    if method == 'edges':
        samples_per_rest_repetition = np.fix(median_instances / (2*n_classes - 1)).astype('int'); # How many we want to keep for each rest repetition
        if samples_per_rest_repetition < 1:
            samples_per_rest_repetition = 1;
        
        changes = np.diff(y_true) # Stimulus change
        idx_changes = np.nonzero(changes)[0] # Stimulus change
        idx_from_rest = idx_changes[np.arange(start=0,stop=idx_changes.size,step=2)] # Changing from rest to movement
        idx_to_rest = idx_changes[np.arange(start=1,stop=idx_changes.size,step=2)] # Changing from rest to movement
        idx_to_rest = np.hstack(([0], idx_to_rest))
        
        idx_keep = []
        for ii,jj in zip(idx_to_rest,idx_from_rest):
            center = np.fix(ii + (jj-ii)/2)
            idx_keep.extend(np.arange(center,center+samples_per_rest_repetition))
        idx_keep = np.asarray(idx_keep, dtype='int')
        idx_final = np.sort(np.hstack((idx_keep, idx_else)))
    
    true_new = y_true[idx_final]
    pred_new = y_pred[idx_final]
    
    return accuracy_score(true_new, pred_new)

def balanced_log_loss(y_true, y_pred, method = 'edges', random_state=None):
    """Balanced log-loss metric (multi-class).
    Keeps only a subset of the data instances corresponding to the rest class.
    The size of the subset is equal to the median group size of the other 
    classes."""
    
#    y_true = np.asarray(y_true)
#    y_pred = np.asarray(y_pred)
    classes, n_instances = np.unique(y_true, return_counts=True)
    median_instances = np.median(n_instances[1:])
    n_classes = classes.size
    
    idx_rest = np.where(y_true == 0)[0] # Find rest instances
    idx_else = np.where(y_true != 0)[0] # Find all other instances
    
    if method == 'random':
		if random_state is not None:
			np.random.seed(random_state)
		idx_keep = np.random.choice(idx_rest,median_instances, replace=False) # Keep a random subset
		idx_final = np.sort(np.hstack((idx_keep, idx_else)))

    
    if method == 'edges':
        samples_per_rest_repetition = np.fix(median_instances / (2*n_classes - 1)).astype('int'); # How many we want to keep for each rest repetition
        if samples_per_rest_repetition < 1:
            samples_per_rest_repetition = 1;
        
        changes = np.diff(y_true) # Stimulus change
        idx_changes = np.nonzero(changes)[0] # Stimulus change
        idx_from_rest = idx_changes[np.arange(start=0,stop=idx_changes.size,step=2)] # Changing from rest to movement
        idx_to_rest = idx_changes[np.arange(start=1,stop=idx_changes.size,step=2)] # Changing from rest to movement
        idx_to_rest = np.hstack(([0], idx_to_rest))
        
        idx_keep = []
        for ii,jj in zip(idx_to_rest,idx_from_rest):
            center = np.fix(ii + (jj-ii)/2)
            idx_keep.extend(np.arange(center,center+samples_per_rest_repetition))
        idx_keep = np.asarray(idx_keep, dtype='int')
        idx_final = np.sort(np.hstack((idx_keep, idx_else)))
    
    true_new = y_true[idx_final]
    pred_new = y_pred[idx_final]
    
    return log_loss(true_new, pred_new)
    
    