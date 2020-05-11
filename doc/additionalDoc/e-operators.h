/**
\page subp_operators Operators on signals

\section subp_unitary_op Unitary Algebraic operators

\subsection ssubp_unitary_op Vector selector

This entity output a vector remapping an input vector
\f${\bf v}_{in}=[v_0,v_1,\cdots,v_n]\f$.
It is realized by specifying bounds such as \f$({[i,j],[k,l]})\f$,
then the output vector will be the contanetion of the
intervals extracted from the input vector:
\f${\bf v}_{out}=[v_i,v_{i+1},\cdots,v_{j-1},v_{j},v_{k},
v_{k+1},\cdots,v_{l-1},v_l]\f$
For instance if we have an input vector
such that:
\code
1
2
3
4
5
6
7
9
10
\endcode
then specifying the bounds \f$(3,5)\f$ and \f$( 7,10\f$) will gives
the following output vector
\code
3
4
7
8
9
\endcode

 */
