/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2007
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      sotExceptionAbstract.cpp
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



#include <sot-core/exception-abstract.h>
#include <sot-core/debug.h>

using namespace std;

/* ------------------------------------------------------------------------- */
/* --- CONSTRUCTORS -------------------------------------------------------- */
/* ------------------------------------------------------------------------- */

const std::string sotExceptionAbstract::EXCEPTION_NAME = "Abstract";


sotExceptionAbstract::
sotExceptionAbstract (const int& _code,
		      const string & _msg)
    :
    code (_code),
    message (_msg)

{
    return ;

}

/* ------------------------------------------------------------------------ */
/* --- ACCESSORS ---------------------------------------------------------- */
/* ------------------------------------------------------------------------ */

const char *sotExceptionAbstract::
getMessage (void)
{
    return (this->message) .c_str();
}

const string &sotExceptionAbstract::
getStringMessage (void)
{
    return this->message;
}

int sotExceptionAbstract::
getCode (void)
{
    return this->code;
}


/* ------------------------------------------------------------------------- */
/* --- MODIFIORS ----------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
#ifdef SOT_EXCEPTION_PASSING_PARAM 


sotExceptionAbstract::Param& sotExceptionAbstract::Param::
initCopy( const Param& p )
{
    sotDEBUGIN(25);
    if( p.pointersSet )
	{
	    strncpy( function,p.functionPTR,BUFFER_SIZE);
	    strncpy( file,p.filePTR,BUFFER_SIZE);
	    line = p.line;
	    pointersSet=false;
	    set=true;
	} else set=false;
    sotDEBUGOUT(25);
    return *this;
}
sotExceptionAbstract::Param::
Param( const int& _line, const char * _function, const char * _file )
    : functionPTR(_function),line(_line),filePTR(_file),pointersSet(true)
{
    sotDEBUGINOUT(25);
}
#endif //#ifdef SOT_EXCEPTION_PASSING_PARAM 

/* ------------------------------------------------------------------------- */
/* --- OP << --------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */

ostream &
operator << (ostream & os,
	     const sotExceptionAbstract & error)
{
    os << error.getExceptionName()<<"Error [#" << error.code << "]:  " << error.message << endl;

#ifdef SOT_EXCEPTION_PASSING_PARAM 
    if( error.p.set )
	os << "Thrown from "<<error.p.file << ": "<<error.p.function
	   <<" (#"<<error.p.line << ")"<<endl;
#endif //#ifdef SOT_EXCEPTION_PASSING_PARAM 

    return os;
}





/** \file $Source$
 */

/*
 * Local variables:
 * c-basic-offset: 4
 * End:
 */
