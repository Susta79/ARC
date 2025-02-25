# S-Curve
The S-curve consists of 7 sections.
To simplify the calculations, we will consider each section separately. Time and distance start from zero at the beginning of each section. In practice, at the beginning of each section ($t=0$), we will have distance 0 ($d=0$).

## Section 1: constant jerk $J$
In the first section we have constant jerk $J$ and the acceleration increases until it reaches the maximum value A.

$$\tag{j1} \boxed{j_1(t)=J}$$

By integrating the jerk function I get the acceleration:
$$a_1(t)=Jt+Ca_1$$

by imposing the condition of zero acceleration at the beginning, we can calculate the constant $Ca_1$:
$$a_1(t=0)=Ca_1=0$$

$$\tag{a1} \boxed{a_1(t)=Jt}$$

By integrating the acceleration function I get the velocity:
$$v_1(t)=\frac{1}{2}Jt^2+Cv_1$$

by imposing the condition of zero speed at the beginning, we can calculate the constant $Cv_1$:
$$v_1(t=0)=Cv_1=0$$

$$\tag{v1} \boxed{v_1(t)= \frac{1}{2}Jt^2}$$

By integrating the speed function I get the distance:
$$d_1(t)=\frac{1}{6}Jt^3+Cd_1$$

by imposing the condition of zero distance at the beginning, we can calculate the constant $Cd_1$:
$$d_1(t=0)=Cd_1=0$$

$$\tag{d1} \boxed{d_1(t)= \frac{1}{6}Jt^3}$$
At the end of the first section, at time $t=T_1$, the acceleration reaches the maximum value $A$. From this we can derive the value of time T1:
$$\tag{T1} a_1(t=T_1)=A=JT_1 \implies \boxed{T_1=\frac{A}{J}}$$

$$\tag{V1} v_1(t=T_1)=\boxed{V_1=\frac{1}{2}JT_1^2}$$

$$V_1=\frac{1}{2}J\left(\frac{A}{J}\right)^2
=\frac{1}{2}J\frac{A^2}{J^2} \implies
\boxed{V_1=\frac{1}{2}\frac{A^2}{J}}$$

$$\tag{D1} d_1(t=T_1)=\boxed{D_1=\frac{1}{6}JT_1^3}$$

$$D_1=\frac{1}{6}J\left(\frac{A}{J}\right)^3
=\frac{1}{2}J\frac{A^3}{J^3} \implies
\boxed{D_1=\frac{1}{6}\frac{A^3}{J^2}}$$

## Section 2: constant acceleration $A$
Once maximum acceleration $A$ is reached, it is held constant until the time when it is necessary to start decelerating.

$$\tag{j2} \boxed{j_2(t)=0}$$

$$\tag{a2} \boxed{a_2(t)=A}$$

By integrating the acceleration function I get the velocity:
$$v_2(t)=At+Cv_2$$

by imposing the condition of speed $V_1$ at the beginning, we can calculate the constant $Cv_2$:
$$v_2(t=0)=Cv_2=V_1$$

$$\tag{v2} \boxed{v_2(t)=At+V_1}$$

By integrating the speed function I get the distance:
$$d_2(t)=\frac{1}{2}At^2+V_1t+Cd_2$$

by imposing the condition of zero distance at the beginning, we can calculate the constant $Cd_2$:
$$d_2(t=0)=Cd_2=0 $$

$$\tag{d2} \boxed{d_2(t)=\frac{1}{2}At^2+V_1t}$$

by symmetry with the first section, we deduce that the velocity at the end of the second section ($t=T_2$), corresponds to $V-V_1$:
$$v_2(t=T_2)=AT_2+V_1=V-V_1$$

From which we can get the value of time $T_2$:
$$\tag{T2} T_2=\frac{V-2V_1}{A}
=\frac{V}{A}-\frac{\cancel{2}}{A}\frac{1}{\cancel{2}}JT_1^2
=\frac{V}{A}-\frac{\cancel{J}}{\cancel{A}}\frac{A^{\cancel{2}}}{J^{\cancel{2}}}
\implies \boxed{T_2=\frac{V}{A}-\frac{A}{J}}$$

$$\tag{V2} v_2(t=T_2)=\boxed{V_2=AT_2+V_1=V-\frac{1}{2}\frac{A^2}{J}}$$

$$\tag{D2} d_2(t=T_2)=\boxed{D_2=\frac{1}{2}AT_2^2+V_1T_2=\frac{1}{2}V\left(\frac{V}{A}-\frac{A}{J} \right)}$$

## Section 3: constant jerk $-J$
Start the deceleration stretch to reach the maximum speed $V$ at acceleration 0.
$$\tag{j3} \boxed{j_3(t)=-J}$$

By integrating the jerk function I get the acceleration:
$$a_3(t)=-Jt+Ca_3$$

by imposing the condition of acceleration = $A$ at the beginning, we can calculate the constant $Ca_3$:
$$a_3(t=0)=Ca_3=A$$

$$\tag{a3} \boxed{a_3(t)=-Jt+A}$$

By integrating the acceleration function I get the velocity:
$$v_3(t)=-\frac{1}{2}Jt^2+At+Cv_3$$

For continuity with the previous section, we can impose that the velocity at the beginning is worth $V-V1$. From this we derive the constant $Cv_3$:
$$v_3(t=0)=Cv_3=V-V_1$$

$$v_3(t)=-\frac{1}{2}Jt^2+At+V-\frac{1}{2}\frac{A^2}{J}
=V-\frac{1}{2}Jt^2+At-\frac{1}{2}\frac{A^2}{J}$$

Since we can collect part of this equation in this way:
$$-\frac{1}{2}Jt^2+At-\frac{1}{2}\frac{A^2}{J}
=-\frac{1}{2}J\left(t^2-2\frac{A}{J}t+\frac{A^2}{J^2}\right)
=-\frac{1}{2}J\left(\frac{A}{J}-t\right)^2$$

$$\tag{v3} \boxed{v_3(t)=V-\frac{1}{2}J\left(\frac{A}{J}-t\right)^2}$$

By integrating the speed function I get the distance:
$$d_3(t)=Vt-\frac{1}{6}Jt^3-\frac{1}{2}\frac{A^2}{J}t+\frac{1}{2}At^2+Cd_3$$

by imposing the condition of zero distance at the beginning, we can calculate the constant $Cd_3$:
$$d_3(t=0)=Cd_3=0$$

$$\tag{d3} \boxed{d_3(t)=-\frac{1}{6}Jt^3+\frac{1}{2}At^2+\left(V-\frac{1}{2}\frac{A^2}{J} \right)t}$$

At the end of the third section ($T=T_3$), the maximum speed $V$ was reached:
$$v_3(t=T_3)=V-\frac{1}{2}J\left(\frac{A}{J}-T_3\right)^2=V$$

from which we can get the time $T_3$:
$$\tag{T3} \boxed{T_3=\frac{A}{J}=T_1}$$

$$\tag{V3} \boxed{V_3=V}$$

$$d_3(t=T3)=D_3=-\frac{1}{6}JT_1^3+\frac{1}{2}AT_1^2+\left(V-\frac{1}{2}\frac{A^2}{J} \right)T_1$$

$$=VT_1-\frac{1}{6}JT_1^3+\frac{1}{2}AT_1^2-\frac{1}{2}\frac{A^2}{J}T_1$$

$$=VT_1-\frac{1}{6}JT_1^3+\frac{1}{2}A\left(\frac{A}{J}\right)^2-\frac{1}{2}\frac{A^2}{J}\frac{A}{J}$$

$$=VT_1-\frac{1}{6}JT_1^3+\cancel{\frac{1}{2}\frac{A^3}{J^2}}-\cancel{\frac{1}{2}\frac{A^3}{J^2}}$$

$$\tag{D3} \boxed{D_3=VT_1-\frac{1}{6}JT_1^3=V\frac{A}{J}-\frac{1}{6}\frac{A^3}{J^2}}$$

## Section 4: constant speed $V$
Once maximum speed $V$ is reached, it is held constant until the time when it is necessary to start decelerating.

$$\tag{j4} \boxed{j_4(t)=0}$$

$$\tag{a4} \boxed{a_4(t)=0}$$

$$\tag{v4} \boxed{v_4(t)=V}$$

By integrating the speed function I get the distance:
$$d_4(t)=Vt+Cd_4$$

by imposing the condition of zero distance at the beginning, we can calculate the constant $Cd_4$:
$$d_4(t=0)=Cd_4=0$$

$$\tag{d4} \boxed{d_4(t)=Vt}$$

The distance traveled during section 4 depends on the total length to be traveled $L$ minus the distance traveled during acceleration and deceleration:
$$\tag{D4} \boxed{D_4=L-(D_1+D_2+D_3+D_5+D_6+D_7)}$$

$$\tag{T4} \boxed{T_4=\frac{D_4}{V}}$$

## Section 5: constant jerk $-J$

$$\tag{j5} \boxed{j_5(t)=-J}$$

By integrating the jerk function I get the acceleration:
$$a_5(t)=-Jt+Ca_5$$

by imposing the condition of zero acceleration at the beginning, we can calculate the constant $Ca_5$:
$$a_5(t=0)=Ca_5=0$$

$$\tag{a5} \boxed{a_5(t)=-Jt}$$

By integrating the acceleration function I get the velocity:
$$v_5(t)=-\frac{1}{2}Jt^2+Cv_5$$

For continuity with the previous section, we can impose that the velocity at the beginning is worth $V$. From this we derive the constant $Cv_5$:
$$v_5(t=0)=Cv_5=V$$

$$\tag{v5} \boxed{v_5(t)=V-\frac{1}{2}Jt^2}$$

By integrating the speed function I get the distance:
$$d_5(t)=Vt-\frac{1}{6}Jt^3+Cd_5$$

by imposing the condition of zero distance at the beginning, we can calculate the constant $Cd_5$:
$$d_5(t=0)=Cd_5=0$$

$$\tag{d5} \boxed{d_5(t)=Vt-\frac{1}{6}Jt^3}$$

By analogy with section 3, it is inferred that:
$$\tag{T5} \boxed{T_5=T_3=T_1=\frac{A}{J}}$$

$$\tag{V5} \boxed{V_5=V_2=AT_2+V_1}$$

$$\tag{D5} \boxed{D_5=D_3=VT_1-\frac{1}{6}JT_1^3}$$

## Section 6: constant acceleration $-A$
$$\tag{j6} \boxed{j_6(t)=0}$$

$$\tag{a6} \boxed{a_6(t)=-A}$$

By integrating the acceleration function I get the velocity:
$$v_6(t)=-At+Cv_6$$

by imposing the condition of speed $V-V_1$ at the beginning, we can calculate the constant $Cv_6$:
$$v_6(t=0)=Cv_6=V-V_1$$

$$\tag{v6} \boxed{v_6(t)=V-V_1-At}$$

By integrating the speed function I get the distance:
$$d_6(t)=(V-V_1)t-\frac{1}{2}At^2+Cd_6$$

by imposing the condition of zero distance at the beginning, we can calculate the constant $Cd_6$:
$$d_6(t=0)=Cd_6=0 $$

$$\tag{d6} \boxed{d_6(t)=(V-V_1)t-\frac{1}{2}At^2}$$

By analogy with section 2, it is inferred that:
$$\tag{T6} \boxed{T_6=T_2=\frac{V}{A}-\frac{A}{J}}$$

$$\tag{V6} \boxed{V_6=V_1=\frac{1}{2}JT_1^2}$$

$$\tag{D6} \boxed{D_6=D_2=\frac{1}{2}V\left(\frac{V}{A}-\frac{A}{J}\right)}$$


## Section 7: constant jerk $J$

$$\tag{j7} \boxed{j_7(t)=J}$$

By integrating the jerk function I get the acceleration:
$$a_7(t)=Jt+Ca_7$$

by imposing the condition of acceleration = $-A$ at the beginning, we can calculate the constant $Ca_7$:
$$a_7(t=0)=Ca_7=-A$$

$$\tag{a7} \boxed{a_7(t)=Jt-A}$$

By integrating the acceleration function I get the velocity:
$$v_7(t)=-\frac{1}{2}Jt^2-At+Cv_7$$

by imposing the condition of speed = $V_1$ at the beginning, we can calculate the constant $Cv_7$:
$$v_7(t=0)=Cv_7=V_1$$

$$\tag{v7} \boxed{v_7(t)=\frac{1}{2}Jt^2-At+V_1=\frac{1}{2}Jt^2-At+\frac{1}{2}JT_1^2}$$

By integrating the speed function I get the distance:
$$d_7(t)=\frac{1}{6}Jt^3-\frac{1}{2}At^2+\frac{1}{2}JT_1^2t+Cd_7$$

by imposing the condition of zero distance at the beginning, we can calculate the constant $Cd_7$:
$$d_7(t=0)=Cd_7=0$$

$$\tag{d7} \boxed{d_7(t)=\frac{1}{6}Jt^3-\frac{1}{2}At^2+\frac{1}{2}JT_1^2t}$$

By analogy with section 1, it is inferred that:
$$\tag{T7} \boxed{T_7=T_1=\frac{A}{J}}$$

$$\tag{V7} \boxed{V_7=0}$$

$$\tag{D7} \boxed{D_7=D_1=\frac{1}{6}\frac{A^3}{J^2}}$$

## Special case $D_4\le0$
$$D_4=L-(D_1+D_2+D_3+D_5+D_6+D_7)=L-2(D_1+D_2+D_3)=0$$

$$L-2\left(\cancel{\frac{1}{6}\frac{A^3}{J^2}}+\frac{1}{2}V\left(\frac{V}{A}-\frac{A}{J}\right)+V\frac{A}{J}-\cancel{\frac{1}{6}\frac{A^3}{J^2}} \right)=0$$

$$L-2\left(\frac{1}{2}\frac{V^2}{A}-\frac{1}{2}V\frac{A}{J}+V\frac{A}{J} \right)=0$$

$$L-\cancel{2}\left(\frac{1}{\cancel{2}}\frac{V^2}{A}+\frac{1}{\cancel{2}}V\frac{A}{J} \right)=0$$

$$\frac{1}{A}V^2+\frac{A}{J}V-L=0$$

$$V_{1,2}=\frac{-\frac{A}{J}\pm\sqrt{\left(\frac{A}{J} \right)^2+4\frac{L}{A}}}{\frac{2}{A}}$$

$$V_1=\frac{-\frac{A}{J}+\sqrt{\left(\frac{A}{J} \right)^2+4\frac{L}{A}}}{\frac{2}{A}}$$

$$\cancel{V_2=\frac{-\frac{A}{J}-\sqrt{\left(\frac{A}{J} \right)^2+4\frac{L}{A}}}{\frac{2}{A}}}$$