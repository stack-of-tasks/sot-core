# -*- coding: utf-8 -*-
# Copyright 2011, Florent Lamiraux, Thomas Moulard, JRL, CNRS/AIST
#

from functools import reduce

from dynamic_graph import plug
from dynamic_graph.entity import Entity
from dynamic_graph.signal_base import SignalBase
from dynamic_graph.sot.core import Flags
from dynamic_graph.sot.core.feature_point6d_relative import FeaturePoint6dRelative

# Identity matrix of order 4
I4 = reduce(lambda m, i: m + (i * (0.0,) + (1.0,) + (3 - i) * (0.0,),), range(4), ())


class FeaturePositionRelative(Entity):
    """
    Relative position of two rigid-body frames in space as a feature

    Express the position of the \"other\" frame in the \"base\" frame.
    The desired relative position is implicitely defined by input
    signals \"baseReference\" and \"otherReference\". The positions and
    Jacobians of the base and other frames are provided as input by
    signals \"basePosition\" and \"JqBase\", \"otherPosition\", \"JqOther\".

    Input of constructor:
      name:           name of the feature,
      basePosition:   MatrixHomo or signal<MatrixHomo> defining the position of
                      the base frame,
      baseJacobian:   signal<Matrix> defining the jacobian of the base frame
                      position with respect to the robot configuration,
      otherPosition:  MatrixHomo or signal<MatrixHomo> defining the position of
                      the other frame,
      otherJacobian:  signal<Matrix> defining the jacobian of the other frame
                      position with respect to the robot configuration,
      otherReference: MatrixHomo or signal<MatrixHomo> defining the reference
                      position of the other frame,
      baseReference:  MatrixHomo or signal<MatrixHomo> defining the reference
                      position of the base frame,


      Members containing a signal:
        basePosition   : position of base frame: input signal (MatrixHomo),
        otherPosition  : position of other frame: input signal (MatrixHomo),
        otherReference : reference position of the other frame:
                         input signal (MatrixHomo),
        baseReference  : reference position of the base frame:
                         input signal (MatrixHomo),
        JqOther        : Jacobian of other frame wrt robot configuration:
                         input signal (Matrix),
        JqBase         : Jacobian of base frame wrt robot configuration:
                         input signal (Matrix),
                           o -1      o*
        error          :  M      .  M
                         b         b
        jacobian       : jacobian output signal with respect to the robot
                         configuration,
        selec:         : selection flag "RzRyRxTzTyTx" (string).
    """

    signalMap = dict()

    def __init__(
        self,
        name,
        basePosition=None,
        otherPosition=None,
        baseReference=None,
        otherReference=None,
        JqBase=None,
        JqOther=None,
    ):
        self._feature = FeaturePoint6dRelative(name)
        self.obj = self._feature.obj
        self._reference = FeaturePoint6dRelative(name + "_ref")
        # Set undefined input parameters as identity matrix
        if basePosition is None:
            basePosition = I4
        if otherPosition is None:
            otherPosition = I4
        if baseReference is None:
            baseReference = I4
        if otherReference is None:
            otherReference = I4

        # If input positions are signals, plug them, otherwise set values
        for (sout, sin) in (
            (basePosition, self._feature.signal("positionRef")),
            (otherPosition, self._feature.signal("position")),
            (baseReference, self._reference.signal("positionRef")),
            (otherReference, self._reference.signal("position")),
        ):
            if isinstance(sout, SignalBase):
                plug(sout, sin)
            else:
                sin.value = sout

        if JqBase:
            plug(JqBase, self._feature.signal("JqRef"))
        if JqOther:
            plug(JqOther, self._feature.signal("Jq"))
        self._feature.setReference(self._reference.name)
        self._feature.signal("selec").value = Flags("111111")
        self._feature.frame("current")

        # Signals stored in members
        self.basePosition = self._feature.signal("positionRef")
        self.otherPosition = self._feature.signal("position")
        self.baseReference = self._reference.signal("positionRef")
        self.otherReference = self._reference.signal("position")
        self.JqBase = self._feature.signal("JqRef")
        self.JqOther = self._feature.signal("Jq")
        self.error = self._feature.signal("error")
        self.jacobian = self._feature.signal("jacobian")
        self.selec = self._feature.signal("selec")

        self.signalMap = {
            "basePosition": self.basePosition,
            "otherPosition": self.otherPosition,
            "baseReference": self.baseReference,
            "otherReference": self.otherReference,
            "JqBase": self.JqBase,
            "JqOther": self.JqOther,
            "error": self.error,
            "jacobian": self.jacobian,
            "selec": self.selec,
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
            raise RuntimeError("No signal with this name")

    def signals(self):
        """
        Return the list of signals
        """
        return self.signalMap.values()

    def commands(self):
        """
        Return the list of commands.
        """
        return self._feature.commands()

    def frame(self, f):
        return self._feature.frame(f)

    def keep(self):
        return self._feature.keep()
