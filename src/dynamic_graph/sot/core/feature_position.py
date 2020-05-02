# -*- coding: utf-8 -*-
# Copyright 2011, Florent Lamiraux, Thomas Moulard, JRL, CNRS/AIST
#

from dynamic_graph import plug
from dynamic_graph.entity import Entity
from dynamic_graph.sot.core.feature_point6d import FeaturePoint6d


class FeaturePosition(Entity):
    """
    Position of a rigid-body in space as a feature

      Input:
        a string: name of the feature,
        a signal<MatrixHomo>: defining the value of the feature,
        a signal<Matrix>: defining the jacobian of the feature with respect
        to the robot configuration,
        an homogeneous matrix: defining the reference value of the feature.

      Members containing a signal:
        position:  position input signal (MatrixHomo),
        reference: reference position input signal (MatrixHomo),
        Jq:        Jacobian input signal (Matrix),
        selec:    selection flag "RzRyRxTzTyTx" (string).
    """

    signalMap = dict()

    def __init__(self,
                 name,
                 signalPosition=None,
                 signalJacobian=None,
                 referencePosition=None):
        self._feature = FeaturePoint6d(name)
        self.obj = self._feature.obj
        self._reference = FeaturePoint6d(name + '_ref')
        if referencePosition:
            self._reference.signal('position').value = tuple(referencePosition)
        if signalPosition:
            plug(signalPosition, self._feature.signal('position'))
        if signalJacobian:
            plug(signalJacobian, self._feature.signal('Jq'))
        self._feature.setReference(self._reference.name)
        self._feature.signal('selec').value = '111111'
        self._feature.frame('current')

        # Signals stored in members
        self.position = self._feature.signal('position')
        self.reference = self._reference.signal('position')
        self.velocity = self._reference.signal('velocity')
        self.Jq = self._feature.signal('Jq')
        self.error = self._feature.signal('error')
        self.errordot = self._feature.signal('errordot')
        self.selec = self._feature.signal('selec')

        self.signalMap = {
            'position': self.position,
            'reference': self.reference,
            'Jq': self.Jq,
            'error': self.error,
            'selec': self.selec
        }

    @property
    def name(self):
        return self._feature.name

    def signal(self, name):
        """
        Get a signal of the entity from signal name
        """
        if name in self.signalMap.keys():
            return self.signalMap[name]
        else:
            raise RuntimeError('No signal with this name')

    def signals(self):
        """
        Return the list of signals
        """
        return self.signalMap.values()

    def commands(self):
        """
        Return the list of commands.
        """
        return ('frame', 'getFrame', 'keep')

    def frame(self, f):
        return self._feature.frame(f)

    def getFrame(self):
        return self._feature.getFrame()

    def keep(self):
        return self._feature.keep()
