Turret Limelight PIDF
$$f(t)=K_pe(t)+K_i\int_{0}^{t}e(t)+K_d\frac{\Delta e(t)}{\Delta t}+K_s*signum(e(t))$$

where $K_p$, $K_i$, $K_d$, $K_s$ are constants, $e(t)$ is the error function and $signum(x)$ is a function that returns the sing of a given number

Turret motion profile + FF + PD
$$f(t)=K_s*signum(ref_v(t))+K_v*ref_v(t)+K_a*ref_a(t)+K_pe_p(t)+K_de_v(t)$$

where $K_s$, $K_v$, $K_a$ are feed forward constants (open loop),
$K_p$, $K_d$ are PD contoller constants (closed loop), $e_p(t)$, $e_v(t)$ are the error functions of position and velocity respectively and 
$ref_p(t)$, $ref_v(t)$, $ref_a(t)$ are the target values of position, velocity, acceleration respectively for a given time ($t$)