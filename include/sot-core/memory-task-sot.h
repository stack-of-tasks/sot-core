/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Gepetto, LAAS, CNRS, 2009
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      memory-task-sot.h
 * Project:   SOT
 * Author:    Nicolas Mansard
 *
 * Version control
 * ===============
 *
 *  $Id$
 *
 * Description
 * ============
 *
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


#ifndef __SOT_MEMORY_TASK_HH
#define __SOT_MEMORY_TASK_HH


#include <sot-core/task-abstract.h>

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#ifndef SOTSOT_CORE_EXPORT
# if defined (WIN32) 
#  if defined (sotSOT_CORE_EXPORTS) 
#    define SOTSOT_CORE_EXPORT __declspec(dllexport)
#  else  
#    define SOTSOT_CORE_EXPORT __declspec(dllimport)
#  endif 
# else
#  define SOTSOT_CORE_EXPORT
# endif
#endif

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class SOTSOT_CORE_EXPORT sotMemoryTaskSOT
: public sotTaskAbstract::sotMemoryTaskAbstract, public Entity
{
 public://   protected:
  /* Internal memory to reduce the dynamic allocation at task resolution. */
  ml::MatrixSvd Jt;  //( nJ,mJ );
  ml::Matrix Jp;
  ml::Matrix PJp;

  ml::Matrix Jff; //( nJ,FF_SIZE ); // Free-floating part
  ml::Matrix Jact; //( nJ,mJ );     // Activated part
  ml::Matrix JK; //(nJ,mJ);

  ml::Matrix U,V;
  ml::Vector S;

 public:
  /* mJ is the number of actuated joints, nJ the number of feature in the task,
   * and ffsize the number of unactuated DOF. */
  sotMemoryTaskSOT( const std::string & name,const unsigned int nJ=0,
                    const unsigned int mJ=0,const unsigned int ffsize =0 );

  virtual void initMemory( const unsigned int nJ,
                           const unsigned int mJ,
                           const unsigned int ffsize );

 public: /* --- ENTITY INHERITANCE --- */
  static const std::string CLASS_NAME;
  virtual void display( std::ostream& os ) const;
  virtual const std::string& getClassName( void ) const { return CLASS_NAME; }

 public: /* --- SIGNALS --- */
  Signal< ml::Matrix,int > jacobianInvSINOUT;
  Signal< ml::Matrix,int > jacobianConstrainedSINOUT;
  Signal< ml::Matrix,int > jacobianProjectedSINOUT;
  Signal< ml::Matrix,int > singularBaseImageSINOUT;
  Signal< unsigned int,int > rankSINOUT;

 public: /* --- PARAMS --- */
  virtual void commandLine( const std::string& cmdLine,std::istringstream& cmdArgs,
			    std::ostream& os );
};

#endif // __SOT_MEMORY_TASK_HH
