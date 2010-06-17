# 
#  Copyright 
# 

SET(${PROJECT_NAME}_HEADERS
sot-core-api.h

#debug
sotContiifstream.h
sotDebug.h

#exception
sotExceptionAbstract.h
sotExceptionDynamic.h
sotExceptionFactory.h
sotExceptionFeature.h
sotExceptionSignal.h
sotExceptionTask.h
sotExceptionTools.h
sotExceptionTraces.h

#signal
sotSignalCast.h

#matrix
sotBinaryOp.h
sotDerivator.h
sotFIRFilter.h
sotIntegratorAbstract.h
sotIntegratorEuler.h
sotMatrixConstant.h
sotUnaryOp.h
sotVectorConstant.h
sotVectorToRotation.h

#factory
sotAdditionalFunctions.h
sotFactory.h
sotMacrosSignal.h
sotPool.h

#math
sotMatrixForce.h
sotMatrixHomogeneous.h
sotMatrixRotation.h
sotMatrixTwist.h
sotOpPointModifior.h
sotVectorQuaternion.h
sotVectorRollPitchYaw.h
sotVectorRotation.h
sotVectorUTheta.h

#feature
sotFeaturePoint6d.h
sotFeatureVector3.h
sotFeatureAbstract.h
sotFeatureGeneric.h
sotFeatureJointLimits.h
sotFeature1D.h
sotFeaturePoint6dRelative.h
sotFeatureVisualPoint.h
sotFeatureTask.h
sotFeatureLineDistance.h

#task
sotTaskAbstract.h
sotTaskUnilateral.h
sotTaskPD.h
sotTaskConti.h
sotMultiBound.h
sotConstraint.h
sotGainAdaptative.h
sotTask.h
sotGainHyperbolic.h

#sot
sotFlags.h
sotSOTQr.h
sotMemoryTaskSOT.h
sotSOTH.h
sotSOT.h
sotRotationSimple.h
sotWeightedSOT.h
sotSolverHierarchicalInequalities.h
)

# Recreate correct path for the headers
#--------------------------------------

SET(fullpath_${PROJECT_NAME}_HEADERS)
FOREACH(lHeader ${${PROJECT_NAME}_HEADERS})
  SET(fullpath_${PROJECT_NAME}_HEADERS 
    ${fullpath_${PROJECT_NAME}_HEADERS}
    ./${PROJECT_NAME}/${lHeader}
  )

ENDFOREACH(lHeader)

#----------------------------------------------------
# Install procedure for the header files
#----------------------------------------------------
INSTALL(FILES ${fullpath_${PROJECT_NAME}_HEADERS}
	DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}
        PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
)
