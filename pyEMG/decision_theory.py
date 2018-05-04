# -*- coding: utf-8 -*-
"""
Created on Wed Jun 29 17:22:43 2016

@author: Agamemnon
"""

import numpy as np
from sklearn.metrics import roc_curve, auc

class RocThreshold(object):
    """TODO: docstring"""
    
    def __init__(self, n_classes=None, method=None, drop_intermediate=None, fpr_threshold=None):
        self.method = 'max_random' if method == None else method
        self.drop_intermediate = False if drop_intermediate == None else drop_intermediate
        self.fpr_threshold = 1e-3 if fpr_threshold == None else fpr_threshold
        self.n_classes_ = n_classes
        self.classes_ = None
        self.thresholds_ = dict()
        self.tpr_ = dict()
        self.fpr_ = dict()
        self.roc_auc_ = dict()
        self.optimal_threshold_ = dict()
    
    def fit(self, y_true, y_pred):
        """ Compute class-specific TPR, FPR and find optimal ROC thresholds.
        
        Parameters
        ----------
        
        y_true : array-like or label indicator matrix
            Ground truth (correct) labels for n_samples samples.
        y_pred : array-like of float, shape = (n_samples, n_classes)
            Predicted probabilities, as returned by a classifier’s predict_proba method.

        Attributes
        ----------
        
        """
        
        self.classes_ = np.unique(y_true)
        if self.n_classes_ == None:
            self.n_classes_ = self.classes_.size
        else:
            if self.classes_.size != self.n_classes_:
                raise ValueError('Number of defined classes not compatible with classes in target vector.')
        
        for i, class_ in enumerate(self.classes_):
            idx = np.where(y_true == class_) # Find the instances belonging to the class
            y_onevsall = np.zeros_like(y_true) # Convert to one-vs-all classifier
            y_onevsall[idx] = 1 # Binary vector for each class
            y_pred_onevsall = y_pred[:,i] # Estimated probabilities for the instances belonging to the class
            self.fpr_[i], self.tpr_[i], self.thresholds_[i] = roc_curve(y_onevsall, y_pred_onevsall, drop_intermediate=self.drop_intermediate)
            self.roc_auc_[i] = auc(self.fpr_[i], self.tpr_[i])         
        
        if self.method == 'max_random':
            self._compute_threshold_max_random()
        elif self.method == 'min_perfect':
            self._compute_threshold_min_perfect()
        elif self.method == 'custom':
            self._compute_threshold_custom()
        else:
            raise ValueError('Unrecognized method.')
        
    def _compute_threshold_max_random(self):
        for i, class_ in enumerate(self.classes_):
            rnd_clf_tpr = np.linspace(0,1,self.thresholds_[i].size)
            self.optimal_threshold_[i] = self.thresholds_[i][np.argmax(self.tpr_[i]-rnd_clf_tpr)]

    def _compute_threshold_min_perfect(self):
        for i, class_ in enumerate(self.classes_):
            self.optimal_threshold_[i] = self.thresholds_[i][np.argmin(np.sqrt((self.tpr_[i]-1)**2 + (self.fpr_[i]-0)**2))]
    
    def _compute_threshold_custom(self):
        """Select the lowest threshold for which false positive rate is larger than a threshold.
        For the rest class, set the threshold to 0. Maximum allowed threshold value is 0.999. """
        for i, class_ in enumerate(self.classes_):
            turning_point = np.where(self.fpr_[i]>self.fpr_threshold)[0][0]
            class_threshold = self.thresholds_[i][turning_point]
            if class_threshold < 0.995:
                self.optimal_threshold_[i] = class_threshold
            else:
                self.optimal_threshold_[i] = 0.995
            
        
                
def control_action(state_old, prediction, prediction_proba, threshold):
    """Decide whether to move to a new state or stick with the old one."""
    if (state_old != prediction) and (prediction_proba > threshold):
        state_new = prediction
    else:
        state_new = state_old
    return state_new


        
        
        