# To do:

- To use Boost.asio to do the socket communication.
- IK: check the joint limits
- Complete the error_def file
- Add the header on all file, with project, descrption and all.

Bug da risolvere
- Comunicazione Tcp. Se la chiudo poi non riesco più a ri-attivarla.
- Unity rispedisce le coordinate ricevute come risposta invece di leggere le coordinate reali del robot.

Cinematica
1. movimento PTP con profilo di movimento a S (curva a 7 segmenti).
2. movimento lineare di sola traslazione (A, B e C constanti) con profilo di movimento a S (curva a 7 segmenti). Lontano da singolarità.
3. movimento lineare completo (rototraslazione) con profilo di movimento a S (curva a 7 segmenti). Lontano da singolarità.
4. movimento lineare completo (rototraslazione) con profilo di movimento Bezier. Lontano da singolarità.



## 2025.02.10
Split the slerp of the circular path into 2 different parts (maybe 1 or 3 parts depending on the angle).

## 2025.02.12
Implement the spline path using the polinomial and the DeCasteliau formula.

Implement the 3 different types of trajectory:
- S-Curve -> Done but to be completed with the special cases.
- Sinusoidal
- Bezier

Optimal trajectory