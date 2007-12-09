
%module playerc

%{
#include "playerc.h"
%}

%include "typemaps.i"

// Special rules for functions that return multiple results via pointers

// For playerc_simulation_get_pose2d()
%apply double *OUTPUT { double *x, double *y, double *a };

// Provide array (write) access
%typemap(in) double [ANY] (double temp[$1_dim0])
{
  int i;
  if (!PySequence_Check($input))
  {
    PyErr_SetString(PyExc_ValueError,"Expected a sequence");
    return NULL;
  }
  if (PySequence_Length($input) != $1_dim0) {
    PyErr_SetString(PyExc_ValueError,"Size mismatch. Expected $1_dim0 elements");
    return NULL;
  }
  for (i = 0; i < $1_dim0; i++)
  {
    PyObject *o = PySequence_GetItem($input,i);
    if (PyNumber_Check(o))
    {
      temp[i] = (float) PyFloat_AsDouble(o);
    }
    else
    {
      PyErr_SetString(PyExc_ValueError,"Sequence elements must be numbers");
      return NULL;
    }
  }
  $1 = temp;
}

// typemap for passing points into the graphics2d interface
%typemap(python,in) player_point_2d_t pts[]
{
	// Check if is a list
	if (PyList_Check ($input))
	{
		int size = PyList_Size ($input);
		int ii = 0;
		$1 = (player_point_2d_t*) malloc (size * sizeof (player_point_2d_t));
		for (ii = 0; ii < size; ii++)
		{
			PyObject *o = PyList_GetItem ($input, ii);
			if (PyTuple_Check (o))
			{
				if (PyTuple_GET_SIZE (o) != 2)
				{
					PyErr_SetString (PyExc_ValueError, "tuples must have 2 items");
					free ($1);
					return NULL;
				}
				$1[ii].px = PyFloat_AsDouble (PyTuple_GET_ITEM (o, 0));
				$1[ii].py = PyFloat_AsDouble (PyTuple_GET_ITEM (o, 1));
			}
			else
			{
				PyErr_SetString (PyExc_TypeError, "list must contain tuples");
				free ($1);
				return NULL;
			}
		}
	}
	else
	{
		PyErr_SetString (PyExc_TypeError, "not a list");
		return NULL;
	}
}

// typemap for passing 3d points into the graphics3d interface
%typemap(python,in) player_point_3d_t pts[]
{
	// Check if is a list
	if (PyList_Check ($input))
	{
		int size = PyList_Size ($input);
		int ii = 0;
		$1 = (player_point_3d_t*) malloc (size * sizeof (player_point_3d_t));
		for (ii = 0; ii < size; ii++)
		{
			PyObject *o = PyList_GetItem ($input, ii);
			if (PyTuple_Check (o))
			{
				if (PyTuple_GET_SIZE (o) != 3)
				{
					PyErr_SetString (PyExc_ValueError, "tuples must have 3 items");
					free ($1);
					return NULL;
				}
				$1[ii].px = PyFloat_AsDouble (PyTuple_GET_ITEM (o, 0));
				$1[ii].py = PyFloat_AsDouble (PyTuple_GET_ITEM (o, 1));
				$1[ii].pz = PyFloat_AsDouble (PyTuple_GET_ITEM (o, 2));
			}
			else
			{
				PyErr_SetString (PyExc_TypeError, "list must contain tuples");
				free ($1);
				return NULL;
			}
		}
	}
	else
	{
		PyErr_SetString (PyExc_TypeError, "not a list");
		return NULL;
	}
}

// typemap to free the array created in the previous typemap
%typemap(python,freearg) player_point2d_t pts[]
{
	free ((player_point2d_t*) $input);
}

// typemap for tuples to colours
%typemap(python,in) player_color_t (player_color_t temp)
{
	// Check it is a tuple
	if (PyTuple_Check ($input))
	{
		// Check the tuple has four elements
		if (PyTuple_GET_SIZE ($input) != 4)
		{
			PyErr_SetString (PyExc_ValueError, "tuple must have 4 items");
			return NULL;
		}
		temp.alpha = PyInt_AsLong (PyTuple_GET_ITEM ($input, 0));
		temp.red = PyInt_AsLong (PyTuple_GET_ITEM ($input, 1));
		temp.green = PyInt_AsLong (PyTuple_GET_ITEM ($input, 2));
		temp.blue = PyInt_AsLong (PyTuple_GET_ITEM ($input, 3));
	}
	else
	{
		PyErr_SetString (PyExc_TypeError, "not a tuple");
		return NULL;
	}
}

// Provide array (write) access
%typemap(in) double [ANY][ANY] (double temp[$1_dim0][$1_dim1])
{
  int i,j;
  if (!PySequence_Check($input))
  {
    PyErr_SetString(PyExc_ValueError,"Expected a sequence");
    return NULL;
  }
  if (PySequence_Length($input) != $1_dim0) {
    PyErr_SetString(PyExc_ValueError,"Size mismatch. Expected $1_dim0 elements");
    return NULL;
  }
  for (i = 0; i < $1_dim0; i++)
  {
    PyObject *olist = PySequence_GetItem($input,i);
    if (!PySequence_Check(olist))
    {
      PyErr_SetString(PyExc_ValueError,"Expected a sequence");
      return NULL;
    }
    if (PySequence_Length(olist) != $1_dim1) {
      PyErr_SetString(PyExc_ValueError,"Size mismatch. Expected $1_dim1 elements");
      return NULL;
    }

    for (j = 0; j < $1_dim1; j++)
    {
      PyObject *o = PySequence_GetItem(olist,j);

      if (PyNumber_Check(o))
      {
        temp[i][j] = (float) PyFloat_AsDouble(o);
      }
      else
      {
        PyErr_SetString(PyExc_ValueError,"Sequence elements must be numbers");
        return NULL;
      }
    }
  }
  $1 = temp;
}

%typemap(in) uint8_t
{
  $1 = (uint8_t) PyLong_AsLong($input);
}

// Integer types
%typemap(out) uint8_t
{
  $result = PyInt_FromLong((long) (unsigned char) $1);
}

%typemap(out) uint16_t
{
  $result = PyInt_FromLong((long) (unsigned long) $1);
}

%typemap(out) uint32_t
{
  $result = PyInt_FromLong((long) (unsigned long long) $1);
}

// Provide array access
%typemap(out) double [ANY]
{
  int i;
  $result = PyList_New($1_dim0);
  for (i = 0; i < $1_dim0; i++)
  {
    PyObject *o = PyFloat_FromDouble((double) $1[i]);
    PyList_SetItem($result,i,o);
  }
}

%typemap(out) uint8_t [ANY]
{
  int i;
  $result = PyList_New($1_dim0);
  for (i = 0; i < $1_dim0; i++)
  {
    PyObject *o = PyInt_FromLong((long) (unsigned char) $1[i]);
    PyList_SetItem($result,i,o);
  }
}

// Provide array access doubly-dimensioned arrays
%typemap(out) double [ANY][ANY]
{
  int i, j;
  $result = PyList_New($1_dim0);
  for (i = 0; i < $1_dim0; i++)
  {
    PyObject *l = PyList_New($1_dim1);
    for (j = 0; j < $1_dim1; j++)
    {
      PyObject *o = PyFloat_FromDouble((double) $1[i][j]);
      PyList_SetItem(l,j,o);
    }
    PyList_SetItem($result,i,l);
  }
}

// Catch-all rule to converts arrays of structures to tuples of proxies for
// the underlying structs
%typemap(out) SWIGTYPE [ANY]
{
 int i;
  $result = PyTuple_New($1_dim0);
  for (i = 0; i < $1_dim0; i++)
  {
    PyObject *o = SWIG_NewPointerObj($1 + i, $1_descriptor, 0);
    PyTuple_SetItem($result,i,o);
  }
}

// Provide thread-support on some functions
%exception read
{
  Py_BEGIN_ALLOW_THREADS
  $action
  Py_END_ALLOW_THREADS
}


// Include Player header so we can pick up some constants and generate
// wrapper code for structs
%include "../../../../libplayercore/player.h"


// Use this for regular c-bindings;
// e.g. playerc_client_connect(client, ...)
//%include "../../playerc.h"


// Use this for object-oriented bindings;
// e.g., client.connect(...)
// This file is created by running ../parse_header.py
%include "playerc_oo.i"
