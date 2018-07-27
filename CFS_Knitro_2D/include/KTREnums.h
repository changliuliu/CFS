/*******************************************************/
/* Copyright (c) 2015 by Artelys                       */
/* All Rights Reserved                                 */
/*******************************************************/
#pragma once

#include "knitro.h"

namespace knitro {

namespace KTREnums {

  /**
   * Valid values for the objective type. Used when defining any optimization problem.
   */
  namespace ObjectiveType {
    enum {
      ObjGeneral = KTR_OBJTYPE_GENERAL,  //!< Use this option if the objective function is neither linear nor quadratic in the decision variables.
      ObjLinear = KTR_OBJTYPE_LINEAR,  //!< Use this option if the objective function is linear in the decision variables.
      ObjQuadratic = KTR_OBJTYPE_QUADRATIC  //!< Use this option if the objective function is quadratic in the decision variables.
    };
  };

  /**
   * Valid values for the objective and constraint function type. Used when defining an optimization problem with integer variables.
   */
  namespace FunctionType {
    enum {
      Uncertain = KTR_FNTYPE_UNCERTAIN,     //!< Use this option if you cannot determine the convexity of the function.
      Convex = KTR_FNTYPE_CONVEX,      //!< Use this option if the function is convex.
      Nonconvex = KTR_FNTYPE_NONCONVEX  //!< Use this option if the function is non-convex.
    };
  };

  /**
   * Valid values for the objective goal. Used when defining any optimization problem.
   */
  namespace ObjectiveGoal {
    enum {
      Minimize = KTR_OBJGOAL_MINIMIZE,  //!< Use this option to minimize the objective function value.
      Maximize = KTR_OBJGOAL_MAXIMIZE  //!< Use this option to maximize the objective function value.
    };
  };

  /**
   * Valid values for variable types. Used when defining an optimization problem with integer or binary variables.
   */
  namespace VariableType {
    enum {
      Continuous = KTR_VARTYPE_CONTINUOUS,  //!< Variable is continuous.
      Integer = KTR_VARTYPE_INTEGER,      //!< Variable is integer-valued.
      Binary = KTR_VARTYPE_BINARY         //!< Variable is binary.
    };
  };

  /**
   * Valid values for an individual constraint type. Used when defining any optimization problem.
   */
  namespace ConstraintType {
    enum {
      ConGeneral = KTR_CONTYPE_GENERAL,  //!< Use this option if the individual constraint function is neither linear nor quadratic in the decision variables.
      ConLinear = KTR_CONTYPE_LINEAR,  //!< Use this option if the individual constraint function is linear in the decision variables.
      ConQuadratic = KTR_CONTYPE_QUADRATIC  //!< Use this option if the individual constraint function is quadratic in the decision variables.
    };
  };

  /**
  * Valid values for an individual residual type.
  */
  namespace ResidualType {
    enum {
      ResGeneral = KTR_RESTYPE_GENERAL,
      ResLinear = KTR_RESTYPE_LINEAR
    };
  };

};
}

