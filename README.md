# PDE_CONTROL
====================================================================================
PDE_HEAT:
∂u/∂t = α∇²u

where u(x,y,t) is the temperature at position (x,y) and time t, α is the thermal diffusivity of the material, and ∇² is the Laplacian operator.

Suppose we want to control the temperature distribution in a material with a specified boundary temperature profile. We can introduce a boundary control input u_b(x,y,t) that represents the temperature at the boundary, and modify the boundary condition as:

u(x,y,t)|_∂Ω = u_b(x,y,t)

where ∂Ω denotes the boundary of the material.

The goal is to design a control strategy that determines the boundary temperature profile u_b(x,y,t) that achieves a desired temperature distribution in the material. This can be formulated as an optimization problem, where the objective is to minimize a cost function that measures the deviation between the desired and actual temperature distribution subject to the PDE and the boundary condition.

The solution to this problem involves finding the optimal boundary temperature profile u_b(x,y,t) that achieves the desired temperature distribution. This can be done using techniques from optimal control theory, such as the Pontryagin maximum principle or dynamic programming.
====================================================================================
