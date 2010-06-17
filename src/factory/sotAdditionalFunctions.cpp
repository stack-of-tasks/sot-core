/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotAdditionalFunctions.cpp
 * Project:   SOT
 * Author:    François Bleibel
 *
 * Version control
 * ===============
 *
 *  $Id$
 *
 * Description
 * ============
 * SOT-specific functions for the shell
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/



#include <dynamic-graph/interpreter.h>
#include <sot-core/sotDebug.h>
#include <sot-core/sotFactory.h>
#include <sot-core/sotAdditionalFunctions.h>
#include <sot-core/signal-cast.h>
#include <dynamic-graph/all-signals.h>
#include <sot-core/sotFlags.h>
using namespace std;

/* \brief Constructor. At creation, overloads (deregisters-then registers
 * again) the 'new' function in the shell
 */
sotAdditionalFunctions::sotAdditionalFunctions() {
	// overload 'new'
	Shell.deregisterFunction("new");
	Shell.registerFunction("new", &sotAdditionalFunctions::cmdNew);
}

sotAdditionalFunctions::~sotAdditionalFunctions() {
	Shell.deregisterFunction("new");
}

void sotAdditionalFunctions::cmdNew( const std::string& cmdLine, istringstream& cmdArg, std::ostream& os )
{
  if( cmdLine == "help" )
    {
      os << "  - new <class> <object>"
	 << "\t\t\tCreate a new object (entity, task, or feature)." <<endl;
      return;
    }
  string className;
  string objName;
  cmdArg >> className >>objName;
  sotDEBUG(15) << "New <" << className<<"> requested."<<endl;
  if( factory.existEntity( className ) )
    {
      sotDEBUG(15) << "New entity<"<<className<<"> " <<objName<<std::endl;
      factory.newEntity(className,objName);
    }
  else if( sotFactory.existFeature( className ) )
    {
      sotDEBUG(15) << "New feature<"<<className<<"> " <<objName<<std::endl;
      sotFactory.newFeature(className,objName);
    }
  else if( sotFactory.existTask( className ) )
    {
      sotDEBUG(15) << "New Task<"<<className<<"> " <<objName<<std::endl;
      sotFactory.newTask(className,objName);
    }
  else os << "  !! Class <" << className << "> does not exist."<<endl;
}

void sotAdditionalFunctions::
cmdMatrixDisplay( const std::string& cmdLine, std::istringstream& cmdArg, std::ostream& os )
{
   if( cmdLine == "help" )
    {
      os << "  - dispmat {simple|complet|matlab}"
	 << "\t\tChange the display of matrix." <<endl;
      return;
    }

   std::string arg;
   cmdArg >> arg;
//    dgDEBUG(15) << "Display matrix: arg = \""<< arg
// 		<<"\" old="<<0+maal::boost::getDisplayType()<<endl;
//    maal::boost::Vector trojan(3);

//    if( ("simple"==arg)||(arg[0]=='s') )
//       { trojan.setDisplayType( maal::boost::SIMPLE ); }
//    else if( ("matlab"==arg)||(arg[0]=='m') )
//      {
//        trojan.setDisplayType( maal::boost::MATLAB );
//        dgDEBUG(15) << "Display matrix: curr = \""
// 		    <<0+maal::boost::MATLAB<<endl;
//      }
//    else if( "cpp"==arg ) { trojan.setDisplayType( maal::boost::CPP ); }
//    else if( ("complet"==arg)||(arg[0]=='c') )
//      { trojan.setDisplayType( maal::boost::COMPLET ); }
//    else { os << "Arg <" << arg << "> is not a valid option."<<endl; }

//    dgDEBUG(15) << "Display matrix: curr = \""
// 		<<0+maal::boost::getDisplayType()<<endl;
//    dgDEBUGF(15,"Ptr fun: %p.", maal::boost::getDisplayType);
   //   dgDEBUG(15) << trojan<<std::endl;

   if( ("simple"==arg)||(arg[0]=='s') )
     SignalCast<ml::Vector>::displayType = maal::boost::SIMPLE;
   else if( ("matlab"==arg)||(arg[0]=='m') )
     SignalCast<ml::Vector>::displayType = maal::boost::MATLAB;
   else if( "cpp"==arg )
     SignalCast<ml::Vector>::displayType = maal::boost::CPP;
   else if( ("complet"==arg)||(arg[0]=='c') )
     SignalCast<ml::Vector>::displayType = maal::boost::COMPLET;
   else { os << "Arg <" << arg << "> is not a valid option."<<endl; }

}

void sotAdditionalFunctions::
cmdFlagSet( const std::string& cmdLine, istringstream& cmdArg, std::ostream& os )
{
  if( cmdLine == "help" )
    {
      os << "  - set <obj1.sig1(flag type)> {#&|}START:END"
	 << "\t\tSet or reset the flag value." <<endl;
      return;
    }

  try {
    Signal<sotFlags,int> &sig1
      = dynamic_cast< Signal<sotFlags,int>& >( pool.getSignal(cmdArg) );

    dgDEBUG(25) << "set..."<<endl;
    sotFlags fl; try { fl = sig1.accessCopy(); } catch(...) {}
    cmdArg >> std::ws >> fl;
    dgDEBUG(15) << "Fl=" << fl <<std::endl;
    sig1 = fl;

  } catch( ExceptionAbstract & err ) { throw;  }
  catch( ... ) {
    DG_THROW ExceptionFactory( ExceptionFactory::SYNTAX_ERROR,
				   "setflag: sig should be of flag type. ",
				   "(while calling setflag).");
  }

}

namespace {
	sotAdditionalFunctions functions;
	ShellFunctionRegisterer regFun18
   ( "setflag",boost::bind(sotAdditionalFunctions::cmdFlagSet,_1,_2,_3) );
    ShellFunctionRegisterer regFun6
    ( "dispmat",boost::bind(sotAdditionalFunctions::cmdMatrixDisplay,_1,_2,_3) );
}
