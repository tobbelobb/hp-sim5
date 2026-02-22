<!-- Converted from sensors-18-02898.xml and cross-checked against Kinematic_Calibration.pdf -->

---

*Article*

# Kinematic Calibration of a Cable-Driven Parallel Robot for 3D Printing

Sen Qian, Kunlong Bao, Bin Zi *, Ning Wang

**Affiliation**  
School of Mechanical Engineering, Hefei University of Technology, Hefei 230009, China

**Emails (from PDF header)**  
qiansenhfut@126.com; baopinche@163.com; wning@mail.hfut.edu.cn; binzi.cumt@163.com

**Correspondence**  
Correspondence: binzi.cumt@163.com; Tel.: +86-551-6290-1606

**Citation / publication info**  
Sensors 18(9) 2898; DOI: 10.3390/s18092898

**Dates**  
Received: 20 July 2018  
Accepted: 28 August 2018  
Published: 1 September 2018

## Abstract

Three-dimensional (3D) printing technology has been greatly developed in the last decade and gradually applied in the construction, medical, and manufacturing industries. However, limited workspace and accuracy restrict the development of 3D printing technology. Due to the extension range and flexibility of cables, cable-driven parallel robots can be applied in challenging tasks that require motion with large reachable workspace and better flexibility. In this paper, a cable-driven parallel robot for 3D Printing is developed to obtain larger workspace rather than traditional 3D printing devices. A kinematic calibration method is proposed based on cable length residuals. On the basis of the kinematic model of the cable-driven parallel robot for 3D Printing, the mapping model is established among geometric structure errors, zero errors of the cable length, and end-effector position errors. In order to improve the efficiency of calibration measurement, an optimal scheme for measurement positions is proposed. The accuracy and efficiency of the kinematics calibration method are verified through numerical simulation. The calibration experiment based on the motion capture system indicates that the position error of end-effector is decreased to 0.6157 mm after calibration. In addition, the proposed calibration method is effective and verified for measurement positions outside optimal positions set through experiments.

**Keywords:** cable-driven parallel robot; kinematic calibration; error analysis; additive manufacturing


---

# 1. Introduction

Cable-driven parallel robots (CDPRs) are known as a type of parallel robots. In CDPRs, the end-effector is suspended by several flexible cables, taking the place of rigid links in traditional rigid-link parallel robots \[1,2,3\]. Compared with traditional rigid-link parallel robots, CDPRs have much smaller inertia and higher payload to weight ratio, which provides high speed and acceleration of the end-effector \[4\]. In addition, due to the extension range and flexibility of cables, CDPRs can be applied in challenging tasks that require motion with large reachable workspace and better flexibility as well \[5\]. Three-dimensional (3D) printing technology has been greatly developed in the last decade and gradually applied in the construction, medical, and manufacturing industries. Limited workspace and accuracy restrict the development of 3D printing technology \[6,7\]. In this paper, a cable-driven parallel robot for 3D Printing is developed, in order to obtain larger workspace than traditional 3D printing devices.

The accuracy of CDPRs is mainly affected by the kinematic parameter errors. The kinematic parameter errors consist of the geometric parameter errors and the zero errors of the robot joints. It is necessary to compensate the geometric parameter errors and zero errors in order to realize the accurate position control of CDPRs \[8\]. The kinematic calibration is an effective method to improve the end-effector position accuracy by improving the precision of kinematic model. In general, the kinematic calibration can be divided into error modeling, error measurement, error identification, and error compensation \[9,10,11\]. The error measurement and error identification are two important steps in the calibration process. The error measurement is the process of measuring end-effector positions by means of the sensors, and comparing the measured values with the theoretical values. The error identification is a process of analyzing error measurement results based on a predetermined parameter identification equation, aiming to obtain kinematic parameter errors. Thus, the accuracy of the error identification is determined by error measurement, including the accuracy of measuring sensors and the selection of measuring positions. There are some areas in the workspace where the kinematic parameter errors have little influence on the end-effector position errors, while the influence of the measurement sensors noises on the accuracy of the error measurement is greatly increased. Therefore, the accuracy of error measurement can be improved obviously by selecting the end-effector positions with large position errors in the workspace as the measurement positions. According to the difference of measurement methods, the calibration methods can be divided into self-calibration method \[12,13,14,15\] and external calibration method \[16,17,18,19\].

In general, additional sensors in joints are necessary to obtain joint variables with self-calibration methods. In CDPRs, the end-effector is suspended by several flexible cables instead of rigid links in traditional rigid-link parallel robots, the installation of the sensors in robot joints will decrease the transmission accuracy of cables. Therefore, the external calibration method is more widely used for CDPRs. For the kinematic calibration of robots, many scholars have done a lot of research on improving the precision and efficiency of kinematic calibration, and proposed many efficient calibration methods. Joshi and Surianarayan \[20\] developed a six degrees of freedom cable-driven parallel device and proposed a kinematic calibration method based on the rotary angle errors of end platform. During the kinematic calibration, the assembly error of the inclinometers is considered and error angle on the perpendicularity of the two inclinometers is taken as the parameter to be calibrated. Ren et al. \[21\] has proposed a new orientation constraint that keeps the two orientation of the end-effector invariant, and deduces a calibration algorithm using different angle combinations. Lau et al. \[22\] has proposed a kinematic calibration method based on relative cable length measurement that is used to calibrate the initial cable length and end-effector poses of the CDPR. Duan et al. \[23\] employed the calibration of a cable-driven parallel manipulator by the six degrees of freedom laser tracker, meanwhile they verified the correctness and to what extent the calibration improves the system through the trajectory planning and motion control. Mustafa \[24\] developed a novel cable-driven manipulator with seven degrees of freedom, according to the characteristics of its redundant drives, a self-calibration scheme based on the cable joint variables is proposed.

The above calibration methods can achieve an effective calibration, however most of them need to measure a large number of redundant measurement poses in order to solve nonlinear problem in the calibration process. This is a very time-consuming operation process. Therefore, it is necessary to carry out the optimal selection of the measurement poses, which can reach the acceptable accuracy with fewer measurement configurations. The observability of the measurement configurations was used for this purpose. Borm and Menq \[25\] put forward the concept of observability index, which is used to evaluate the observability of the identification Jacobian matrix for kinematic errors, thus providing a theoretical basis for measurement positions optimization. Scholars have different opinions on the specific value of observability index. Borm and Menq \[26\] proposed the observability index based on the product of all non-zero singular values of identification Jacobian matrix. Driels and Pathre \[27\] proposed the condition number which equals the ratio of the largest singular value to the smallest non-zero singular value. Regarding the inverse of the condition number as the observability index, Nahvi et al. \[28\] regarded the minimum value of the singular value of the identification Jacobian matrix as the observability index. Jia et al. \[29\] proposed an optimization model with two observability indexes, which can take the partial features and the global ones of the singular values into account at the same time. With the observability index determined, the additional challenge is to find an effective algorithm to construct a configuration set whose observability index is maximal. Li et al. \[30\] and Zhou et al. \[31\] adopted DETMAX algorithm to construct the configuration set. Wang et al. \[32\] proposed the modified annealing algorithm for the polishing robot. These algorithms improved convergence performance but often led to a low convergence rate. Many recent methods used a large pool of randomly selected configuration candidates, and focused on finding a certain number of optimal configurations within the pool \[33,34\]. These methods usually make improvements on DETMAX algorithm. Because the size of the candidate pool is usually large, a huge number of computations is required to check all candidates in the pool. Therefore, many scholars put forward different methods to choose the smaller candidate pool. Li et al. \[30\] identified the candidate pool of a six DOF serial robot and compared the characteristics of the optimal selection schemes based on different observability indices. Nategh et al. \[35\] demonstrated the distribution of the candidate pool of a rigid parallel mechanism, and calibrated the six degrees of freedom rigid parallel mechanism using a minimum number of measurement configurations. Wang et al. \[36\] proposed an efficient configuration search method that is based on the closed-form mapping from configuration perturbations to singular-value variations. The above researches on the optimal selection of measurement poses provides a systematic implementation method for the optimal poses selection for parallel or serial rigid robots. Due to the unilateral actuating property of cables, some widely used calibration methods cannot be applied in CDPRs directly, which must be modified to meet the special property of cables.

In this paper, a cable-driven parallel robot for 3D Printing is developed. A kinematic calibration method is proposed based on cable length residuals. On the basis of the kinematic model of the cable-driven parallel robot for 3D Printing, the mapping model is established among geometric structure errors, zero errors of cable, and end-effector position errors. In order to improve the efficiency of calibration measurement, an optimal scheme for measurement positions is proposed. The accuracy and efficiency of the kinematics calibration method are verified through simulation. Furthermore, the calibration experiments based on the motion capture system are carried out to demonstrate the high performance and effectiveness of the proposed calibration method.

# 2. Description of Experimental Prototype

In this paper, a novel CDPR for 3D printing is developed, which is designed for large-scale 3D printing. The structure of the CDPR for 3D Printing is shown in <a href="#sensors-18-02898-f001" data-ref-type="fig">Figure 1</a>. The extension range and flexibility of cables can significantly improve the workspace of 3D printing device at a lower manufacturing cost. The CDPR for 3D Printing adopts the Fused Deposition Molding (FDM) technology. The motion of the end-effector is mainly divided into scanning motion in each horizontal layer and lifting movement in vertical direction. Therefore, in order to realize 3D printing, the end-effector should have at least three degrees of freedom (3-DOF). At the same time, for the sake of ensuring the scanning accuracy of the device on each print layer, the rotational motion of the end-effector must be avoided and then the planeness of each printing layer can be ensured.

Considering the above two requirements, the end-effector is driven by three cable groups. Each cable group consists of two parallel cables. A sketch of the CDPR for 3D Printing is shown in <a href="#sensors-18-02898-f002" data-ref-type="fig">Figure 2</a>. According to parallelogram principle, the end-effector cannot rotate around the coordinate axis under the constraint of three cable groups. Therefore, the end-effector only has three translational degrees of freedom, and the stable translation of the end-effector along each coordinate axis are realized under the constraint of the six cables and the follow-up spring, which is used to keep the tension on cables. The upper end of the follow-up spring is connected to the slide rail, which can guarantee that the follow-up spring is always in the vertical direction. Undoubtedly, the elasticity of the cables will affect the printing precision. It is necessary to choose the cables which has the high tensile strength. The modulus of the Kevlar49 cable is 861.9 cN/dtex and the modulus of the stainless steel cable is about 254.4 cN/dtex. Therefore, choosing Kevlar49 cables as the driven cables that will effectively reduce the effect of cable elasticity on printing accuracy.

As shown in <a href="#sensors-18-02898-f002" data-ref-type="fig">Figure 2</a>, one end (*B*<sub>1</sub> ∼ *B*<sub>6</sub>) of six cables (*L*<sub>1</sub> ∼ *L*<sub>6</sub>) is connected to the end-effector, and the cable winds around the fixed pulley set, through the cable outlets (*A*<sub>1</sub> ∼ *A*<sub>6</sub>), the other end is connected with the slider of the linear module which is made up of slide rail and ball screw. Three linear modules are used to pull six cables. The coordinate system of the CDPR for 3D printing is shown in <a href="#sensors-18-02898-f002" data-ref-type="fig">Figure 2</a>. *O**X**Y**Z* is the global coordinate frame which fixed to the center of the base, *O**X*<sub>1</sub>*Y*<sub>1</sub>*Z*<sub>1</sub> is the local coordinate frame fixed to the end-effector.

# 3. Kinematic Error Modeling

## 3.1. Equivalent of Analytical Structure

Before analyzing the kinematics, equivalent model of the CDPR for 3D printing can be established for simplicity. The schematic of the equivalent model is shown in <a href="#sensors-18-02898-f003" data-ref-type="fig">Figure 3</a>. For the presented 3-DOF CDPR, the center *B* of the end-effector is regarded as the equivalent end point. The six cables *L*<sub>*i*</sub> <sub>(*i* = 1, 2, 3, 4, 5, 6)</sub>, can be equivalent to three cables *L*<sub>*k**a*</sub> *<sub>k</sub>* <sub>= (1, 2, 3)</sub>, which facilitate the analysis of kinematic problems without loss of generality. First of all, vector $\\overset{\\rightarrow}{B\_{i}B}$ are established through connecting the connection point *B*<sub>*i*</sub> between cables and end-effector to the centroid of the end-effector *B*. Due to the parallelogram principle, the orientation of the end-effector is always unchanged, therefore, the value of vectors $\\overset{\\rightarrow}{B\_{i}B}$ are invariant, which is only related to the geometric structure of the end-effector. Secondly, in the context of guaranteeing vectors $\\overset{\\rightarrow}{A\_{i}A\_{ka}}$ are the same as vectors $\\overset{\\rightarrow}{B\_{i}B}$, vectors $\\overset{\\rightarrow}{A\_{i}A\_{ka}}$ are established by connecting the cable outlets *A*<sub>1</sub>, of fixed pulley to virtual cable outlets *A*<sub>*k**a*</sub>. Therefore, it can be obtained from the principle of parallelogram that: *L*<sub>1*a*</sub>//*L*<sub>1</sub>//*L*<sub>2</sub>, *L*<sub>*i*</sub>//*L*<sub>3</sub>//*L*<sub>4</sub>, *L*<sub>3*a*</sub>//*L*<sub>5</sub>//*L*<sub>6</sub>. According to the above parallel conditions, we can simplify the kinematic inverse operation of six cables to the kinematic inverse operation of three cables without losing the generality, and then the computational complexity of inverse kinematic analysis is greatly reduced.

## 3.2. Kinematic Error Modeling

There are several factors that reduce the printing accuracy of the CDPR for 3D Printing, including kinematic error, transmission error, nonlinear error of the control system, deformation error, the measurement error of the sensors, and thermal error. The kinematic errors caused by the manufacturing and assembly are the main error sources. The error model is established in order to describe the relationship between the position errors of the end-effector, the geometric parameter errors of the mechanism, and the zero errors of the cable length.

Kinematic analysis is carried out based on the equivalent model shown in <a href="#sensors-18-02898-f004" data-ref-type="fig">Figure 4</a>. *B*(*x*,*y*,*z*) can be represented as the reference point of the local coordinate frame. The Euler angle of the moving coordinate frame is $\\lbrack\\begin{matrix}
0 & 0 & 0 \\\\
\\end{matrix}\\rbrack$, the global coordinates of the virtual cable outlets *A*<sub>*k**a*</sub> are represented as \[*x*<sub>*k**a*</sub>, *y*<sub>*k**a*</sub>, *z*<sub>*k**a*</sub>\], therefore, the length of cables can be obtained as follows:
$$L\_{ka} = L\_{ko} + L\_{kr} = \\sqrt{(x - x\_{ka})^{2} + {(y - y\_{ka})}^{2} + {(z - z\_{ka})}^{2}}$$
where *L*<sub>*k**a*</sub> are the cable lengths from virtual cable outlets to equivalent end point, *B*, *L*<sub>*k**o*</sub> are the cable lengths from virtual cable outlets to the equivalent end point *B* at the initial position, *L*<sub>*k**r*</sub> are cable lengths variable, which are defined as the differences between *L*<sub>*k**o*</sub> and *L*<sub>*k**a*</sub>.

The implicit function form of Equation (1) can be written as:
*f*<sub>*k*</sub>(*x*,*y*,*z*,*x*<sub>*k**a*</sub>,*y*<sub>*k**a*</sub>,*z*<sub>*k**a*</sub>,*L*<sub>*k**o*</sub>+*L*<sub>*k**r*</sub>) = 0

One can obtain from Equation (2) that the kinematic errors of the device consist of the position error of virtual cable outlets that is represented as $(\\begin{matrix}
{dx\_{ka}} & {dy\_{ka}} & {dz\_{ka}} \\\\
\\end{matrix})$, and the error of initial cable length (i.e., the zero errors of the cable length) that can be represented as *d**L*<sub>*k*</sub>, *<sub>k</sub>* <sub>= (1, 2, 3)</sub>. Thus, a total of 12 geometric error sources can be obtained.

Equation (2) can be differentiated to obtain the following equation:
$$\\begin{matrix}
{2(L\_{ko} + L\_{kr})dL\_{k} - 2(x - x\_{ka})dx + 2(x - x\_{ka})dx\_{ka} - 2(y - y\_{ka})dy} \\\\
{+ 2(y - y\_{ka})dy\_{ka} - 2(z - z\_{ka})dz + 2(z - z\_{ka})dz\_{ka} = 0} \\\\
\\end{matrix}$$

Equation (3) can be expanded and written into matrix form. The mapping relationship of geometric parameter errors, zero errors of the cable length and the end-effector position errors can be obtained as follows:
**C****δ** = **D**<sup>*T*</sup>*Δ***q**
where:
$$\\mathbf{\\delta} = {(\\begin{matrix}
{dx} & {dy} & {dz} \\\\
\\end{matrix})}^{T}$$
$$\\begin{matrix}
{\\Delta\\mathbf{q} =} \\\\
{(\\begin{matrix}
\\begin{matrix}
{dx\_{1a}} & {dy\_{1a}} & {dz\_{1a}} \\\\
\\end{matrix} & \\begin{matrix}
{dx\_{2a}} & {dy\_{2a}} & {dz\_{2a}} \\\\
\\end{matrix} & \\begin{matrix}
{dx\_{3a}} & {dy\_{3a}} & \\begin{matrix}
{dz\_{3a}} & {dL\_{1}} & \\begin{matrix}
{dL\_{2}} & {dL\_{3}} \\\\
\\end{matrix} \\\\
\\end{matrix} \\\\
\\end{matrix} \\\\
\\end{matrix})}^{T} \\\\
\\end{matrix}$$
$$\\mathbf{C} = \\begin{pmatrix}
{x - x\_{1a}} & {y - y\_{1a}} & {z - z\_{1a}} \\\\
{x - x\_{2a}} & {y - y\_{2a}} & {z - z\_{2a}} \\\\
{x - x\_{3a}} & {y - y\_{3a}} & {z - z\_{3a}} \\\\
\\end{pmatrix}$$
$$\\mathbf{D} = \\begin{pmatrix}
{x - x\_{1a}} & 0 & 0 \\\\
{y - y\_{1a}} & 0 & 0 \\\\
{z - z\_{1a}} & 0 & 0 \\\\
0 & {x - x\_{2a}} & 0 \\\\
0 & {y - y\_{2a}} & 0 \\\\
0 & {z - z\_{2a}} & 0 \\\\
0 & 0 & {x - x\_{3a}} \\\\
0 & 0 & {y - y\_{3a}} \\\\
0 & 0 & {z - z\_{3a}} \\\\
{L\_{1o} + L\_{1r}} & 0 & 0 \\\\
0 & {L\_{2o} + L\_{2r}} & 0 \\\\
0 & 0 & {L\_{3o} + L\_{3r}} \\\\
\\end{pmatrix}$$
where **δ** is the position error of the end-effector, and *Δ***q** is kinematic error of the CDPR for 3D printing.

The matrix *C* are reversible in nonsingular positions, thus, Equation (4) can be rewritten as follows:
**δ** = **J**<sub>0</sub>*Δ***q**
where
**J**<sub>0</sub> = **C**<sup>−1</sup>**D**<sup>*T*</sup>
where **J**<sub>0</sub> is the identification Jacobian matrix.

# 4. Optimal Selection

During the process of measuring position, the noise of the measurement sensors is inevitable. The observability is considered as a criterion to find out optimal measurement positions where the position errors of the end-effector are most distinguishable. The noise of the measurement sensors slightly affect the error measurement results in optimal measurement positions.

Optimal selection for measurement positions in this section proceeds in two stages. Through analyzing the position errors distribution in the workspace of the CDPR for 3D printing, the preliminary area of the measurement positions are determined. The coordinates of each optimal position are accurately determined through the calculating the observable index of each point in the preliminary selection region.

## 4.1. Analysis of Workspace and Preliminary Selection

The workspace of the CDPR for 3D printing is the position set that the end-effector can reach within the structural frame. An important characteristic of CDPRs is well known, as cables can only be driven by positive tension in order to keep the straight line shape, rather than negative compression. Therefore, the constraint should be satisfied that cables are all in tension (*t*<sub>*k*</sub> \> 0) in the process of calculating the workspace.

Due to the low inertia of the CDPRs rather than rigid link robots, the end-effector can obtain higher acceleration. Therefore, the dynamic constraints caused by acceleration should be taken into account in the analysis of the workspace. The specific mathematical description of acceleration constraints is as follows:
$$\\begin{matrix}
{\\forall a \< a\_{\\max}} & {\\exists\\mathbf{T} \> 0:\\mathbf{A} \\cdot \\mathbf{T} + \\mathbf{W}} \\\\
\\end{matrix} = ma \\cdot \\mathbf{c}$$
where *a*<sub>max</sub> is the scalar value of the maximum acceleration of the end-effector, $\\mathbf{A} = {(\\begin{matrix}
\\mathbf{u}\_{1} & \\mathbf{u}\_{2} & \\mathbf{u}\_{3} \\\\
\\end{matrix})}$ is the structure matrix of the robot, **u**<sub>*k*</sub> *<sub>k</sub>* <sub>= (1, 2, 3)</sub> is unit directional vector of each cable, and **T** = (*t*<sub>1</sub>,*t*<sub>2</sub>,*t*<sub>3</sub>)<sup>*T*</sup> is the tension value of each cable, **W** is the external force on the end-effector, the value of which is about 15 N, *m* is the weight of end-effector, *a* is the acceleration scalar for end-effector, and **c** is the unit directional vector of end-effector acceleration.

As described in <a href="#sec3-sensors-18-02898" data-ref-type="sec">Section 3</a>, the motion of the end-effector is equivalent to the movement under the constraint of three cables and the follow-up spring. The preload of the follow-up spring is far greater than the self-weight of the end-effector. The follow-up spring can be regarded as a special cable whose tension changes with the z value of the end-effector. The direction of the spring is along the z axis of the global coordinate. Therefore, the force balance equation of the CDPR for 3D printing can be expressed as follows:
**A** ⋅ **T** + *k*(*z*<sub>*k*</sub>−*z*) ⋅ **b** − *m**g* ⋅ **b** = *m**a* ⋅ **c**
where $\\mathbf{b} = {(\\begin{matrix}
0 & 0 & 1 \\\\
\\end{matrix})}^{T}$ is the unit directional vector of the follow-up spring tension, *k* is the elastic coefficient of the follow-up spring, *z*<sub>*k*</sub> is the z value of the end-effector in the initial state, Z is the z value of the end-effector, and g is the acceleration of gravity.

Equation (7) can be rewritten as follows:
**A**<sup>′</sup>**T**<sup>′</sup> = *m**a* ⋅ **c**
where
$$\\mathbf{A}^{\\prime} = (\\begin{matrix}
\\mathbf{A} & \\mathbf{b} \\\\
\\end{matrix})$$
$$\\mathbf{T}^{\\prime} = \\begin{pmatrix}
\\mathbf{T} \\\\
{k(z\_{k} - z) - mg} \\\\
\\end{pmatrix}$$

According to Equation (8), the mathematical description of acceleration constraints can be rewritten as follows:
$$\\begin{matrix}
{\\forall a \< a\_{\\max}} & {\\exists\\mathbf{T}^{\\prime} \> 0:\\mathbf{A}^{\\prime} \\cdot \\mathbf{T}^{\\prime}} \\\\
\\end{matrix} = ma \\cdot \\mathbf{c}$$

The solution of equation can be expressed as:
$$\\mathbf{T}^{\\prime} = \\mathbf{A}^{\\prime +} \\cdot ma \\cdot \\mathbf{c} + \\mathbf{X} + {(\\begin{matrix}
{ma} & {ma} & {ma} \\\\
\\end{matrix})}^{T}$$
where **A**<sup>′+</sup> is generalized inverse matrix of **A**<sup>′</sup>, $\\mathbf{X} + {(\\begin{matrix}
{ma\_{\\max}} & {ma\_{\\max}} & {ma\_{\\max}} \\\\
\\end{matrix})}^{T}$ is a vector in null space of **A**<sup>′</sup>, according to matrix theory, $r \\cdot \\lbrack\\mathbf{X} + {(\\begin{matrix}
{ma\_{\\max}} & {ma\_{\\max}} & {ma\_{\\max}} \\\\
\\end{matrix})}^{T}\\rbrack$ is still a vector in null space of **A**<sup>′</sup>, while *r* is an arbitrary constant, therefore, Equation (10) can be transformed into:
$$\\mathbf{T}^{\\prime} = ma \\cdot \\lbrack\\mathbf{A}^{\\prime +} \\cdot \\mathbf{c} + r \\cdot {(\\begin{matrix}
1 & 1 & 1 \\\\
\\end{matrix})}^{T}\\rbrack + r \\cdot \\mathbf{X}$$

The inertial force generated by the acceleration in specific direction will produce a large resolution force on the cables. Hence, it is necessary to apply the solution method of the force-closure workspace \[37\] to the solution of the workspace that is constrained by the acceleration. It can be obtained from Equation (11) that if ∃**X** \> 0, **T**<sup>′</sup> \> 0 can be satisfied while *r* trend to be infinite. According to matrix theory, if ∃**V** ∈ ker (**A**<sup>′</sup>), ∃**X** \> 0 can be satisfied while **V** \> 0. Therefore, the solution conditions of the workspace can be expressed as:
∃**V** ∈ ker (**A**<sup>′</sup>) : **V** \> 0

According to the need of actual printing, in addition to guaranteeing the tension of cables are all under acceleration constraints, the following constraints should be satisfied during the simulation:

\(1\) The rigid frame constraint on the range of the workspace. Specific constraints can be expressed as follows:
$$\\left\\{ \\begin{matrix}
{- l\_{0}/2 \< x \< l\_{0}/2} \\\\
{- l\_{0}\\tan(\\pi/6) \< y \< \\sqrt{3}(l\_{0}/2 - {\|{x - l\_{0}/2}\|})} \\\\
{0 \< z \< l\_{z}} \\\\
\\end{matrix} \\right.$$
where *l*<sub>0</sub> is the side length of a regular triangle with three virtual cable outlets as apexes, and *l*<sub>*z*</sub> is the initial height of the end-effector.

\(2\) The cable length constraints on the range of the workspace. The device is driven by the fixed length cables. When the end-effector is in the initial position, the cable length from the cable outlets to the end-effector is the maximum *l*<sub>max</sub>. The length of each cable at the remaining positions is less than or equal to *l*<sub>max</sub>. Hence, the specific constraints can be expressed as follows:
0 \< *L*<sub>*k**a*</sub> \< *l*<sub>max</sub>

\(3\) The constraint of cable inclination relative to the horizontal plane. When the inclination of the cables *α*<sub>*k*</sub> *<sub>k</sub>* <sub>= (1, 2, 3)</sub> is too small, it can be known from the decomposition of the force that the force on each cable will increase dramatically. Too large cable tension will affect the performance of the motors. The specific constraints are expressed as follows:
$$\\left\\{ \\begin{matrix}
{0 \< \\alpha\_{1} \< \\pi/12} \\\\
{0 \< \\alpha\_{2} \< \\pi/12} \\\\
{0 \< \\alpha\_{3} \< \\pi/12} \\\\
\\end{matrix} \\right.$$

As shown in <a href="#sensors-18-02898-f004" data-ref-type="fig">Figure 4</a>, the whole frame can be sketched as a tri-prism structure. The bottom of the tri-prism is a regular triangle with the side length 600 mm, and the height of the tri-prism is 650 mm. According to the constraints and structural parameters listed above, the workspace of the end-effector is obtained as <a href="#sensors-18-02898-f005" data-ref-type="fig">Figure 5</a>:

The shape of the workspace can be sketched as a tri-prism. The bottom of the tri-prism is a regular triangle with the side length 180 mm and the height of the tri-prism is 240 mm. During the experimental stage, in order to ensure the stability of the CDPR for 3D Printing, the CDPR for 3D Printing is driven by the fixed length cables. However, it results in a smaller workspace. As mentioned above, the cable length and the rigid frame are the main constraints of the workspace. Therefore, the workspace can be expanded through adopting the winches to drive the CDPR for 3D printing or enlarging the scale of the rigid frame. Due to the extension range and flexibility of cables, the workspace can be expended at a low engineering cost.

In order to reduce the search range of optimal measurement positions and obtain the preliminary positions with greater observability, it is necessary to analyze the distribution of position errors in the workspace. Firstly, random geometric errors (from −1 mm to 1 mm) is added to all desired kinematic parameters. The construction volume errors of the end-effector in the workspace are obtained via forward kinematics. The construction volume error of parallel 3D printing device at different height in workspace can be obtained, the results are depicted in <a href="#sensors-18-02898-f006" data-ref-type="fig">Figure 6</a>. The construction volume errors in the workspace of the CDPR for 3D printing can be expressed as follows:
$$\\Delta V = \\sqrt{\\Delta x^{2} + \\Delta y^{2} + \\Delta z^{2}}$$
where *Δ**x* is the position error of end-effector in x direction, *Δ**y* is the position error of end-effector in y direction, and *Δ**z* is the position error of end-effector in z direction.

As shown in <a href="#sensors-18-02898-f006" data-ref-type="fig">Figure 6</a>, the z values of the end-effector are 110 mm, 150 mm, 190 mm, and 230 mm, respectively. It can be seen that the maximum value of the construction volume errors is distributed at the boundary of the each horizontal plane and the structural volume error of the end-effector decreases with the printing height. Based on the principle of selecting the positions with the larger structural volume errors, the boundary region of the z = 110 mm plane in the workspace is regarded as the preliminary selection area of the measurement positions.

## 4.2. Optimal Positions Selection

After the preliminary selection area of the measurement positions is determined, the optimal positions selection can be divided into two steps. Firstly, the candidates of measurement positions should be selected from z = 110 mm. Secondly, an effective algorithm should be applied to construct an optimal position set, the observability index of which is maximal.

As shown in <a href="#sensors-18-02898-f007" data-ref-type="fig">Figure 7</a>a, the z = 110 mm plane in the workspace is an equilateral triangle region with sides equal to 170 mm. In order to facilitate the selection of the candidates of measurement positions, the boundary of the equilateral triangle region is discretized. Through moving the sides of the equilateral triangle inward by 1mm, an embedded equilateral triangle can be obtained. The candidates of measurement positions are distributed along the edges of the two equilateral triangles. The distribution of the position candidates is shown in <a href="#sensors-18-02898-f007" data-ref-type="fig">Figure 7</a>b, 60 candidates of measurement positions are selected.

As mentioned in <a href="#sec3dot2-sensors-18-02898" data-ref-type="sec">Section 3.2</a>, the CDPR for 3D printing has 12 independent kinematic parameter errors, and each measurement position has 3 kinematic constraint equations. In order to accomplish the error identification, the number of measurement positions K is at least 4. Therefore, the optimal selection of the measurement positions in this paper aimed at selecting 4 optimal positions with the maximum observability index from the 60 candidate positions. Four kinds of observability index are mentioned in the introduction. In this paper, the reciprocal of conditional numbers *O*<sub>2</sub>, is chosen as the observability index. Among four kinds of observability index, the calibration algorithms will be led to a high convergence rate while regarding *O*<sub>2</sub> as observability index \[30\]. Regarding the non-zero singular value of the identification Jacobian **J**<sub>0</sub>, as *σ*<sub>*m*</sub> ≤ … ≤ *σ*<sub>1</sub>, the observability index *O*<sub>2</sub>, can be expressed as:
*O*<sub>2</sub> = *σ*<sub>*m*</sub>/*σ*<sub>1</sub>
where *σ*<sub>*m*</sub> is the maximal non-zero singular value of the identification Jacobian, and *σ*<sub>1</sub> is the minimal non-zero singular value of the identification Jacobian.

By calculating the observability index *O*<sub>2</sub>, of the identification Jacobian **J**<sub>0</sub>, in the above 60 candidate positions, the *K*(*K* = 4) positions with the maximum observability index *O*<sub>2</sub>, among the 60 candidate positions are selected. The method of optimal positions selection is presented as follows:

\(1\) **Q**<sub>*c*</sub> is the candidate position set, there are *C* = 60 candidate points in the initial state.

\(2\) **N**<sub>*l*</sub> is the optimal measurement positions set, it concludes a number of *l* = 0 positions in the initial state.

\(3\) Search the *O*<sub>2</sub> maximum point *q*<sub>*l*</sub>, in **Q**<sub>*c*</sub>, using the extremum seeking function.

\(4\) Add *q*<sub>*l*</sub> to **N**<sub>*l*</sub>, *l* = *l* + 1, meanwhile, remove *q*<sub>*l*</sub> from **Q**<sub>*c*</sub>, *C* = *C* − 1

\(5\) Repeat steps 3, 4, until *l* = *K*, stop the cycle.

After the above steps, the result is that the maximum observation index of the optimal positions set is *O*<sub>2max </sub> = 13.99 × 10<sup>2</sup>, the coordinates of each optimal position in **N**<sub>*l*</sub> are shown in <a href="#sensors-18-02898-t001" data-ref-type="table">Table 1</a>.

# 5. Simulation

As mentioned above, the coordinates of the cable outlets and the initial cable length are the main kinematic parameters that determine the printing accuracy of the CDPR for 3D printing. In order to improve the printing accuracy, a kinematic calibration method based on cable length residuals is proposed. Through measuring the end-effector position errors and cable length variables synchronously by the motion capture system, the calibration method can calibrate the coordinates of the cable outlets and the initial cable length. This section mainly focuses on the simulation for the calibration to verify the effectiveness of this method.

## 5.1. Error Identification Model

The main aim of kinematic calibration is to reduce the position errors of end-effector by the way of kinematic parameters compensation. Therefore, it is the key problem of kinematic calibration to establish the error identification model. In this paper, the functional formula for the error identification is established, which realizes the error identification among the position errors of end-effector, geometric parameter errors, and zero errors of cable length.

The difference between the measured and actual values of the end-effector positions can be expressed as follows:
**e**<sub>*k*</sub> = **e**<sub>*m*</sub> − **e**
where **e**<sub>*k*</sub> is the residuals between the measured and the true position coordinates of the end-effector, that is, the measurement noise of the measurement sensor, **e**<sub>*m*</sub> is the measured position coordinates of the end-effector, and **e** is the true position coordinates of the end-effector.

According to Equation (1), the kinematic constraint equation of the CDPR for 3D printing is as follows:
$$L\_{ka} = L\_{ko} + L\_{kr} = \\sqrt{(x - x\_{ka})^{2} + {(y - y\_{ka})}^{2} + {(z - z\_{ka})}^{2}}$$
where
*L*<sub>*k**o*</sub> = *L*<sub>*k**o*</sub><sup>′</sup> + *Δ**l*<sub>*k*</sub>, *L*<sub>*k**r*</sub> = *L*<sub>*k**r*</sub><sup>′</sup> + *ψ*<sub>*r*</sub>, *x* = *x*<sup>′</sup> + *Δ**x* + *e*<sub>*k**x*</sub>
*y* = *y*<sup>′</sup> + *Δ**y* + *e*<sub>*k**y*</sub>, *z* = *z*<sup>′</sup> + *Δ**z* + *e*<sub>*k**z*</sub>, *x*<sub>*k**a*</sub> = *x*<sub>*k**a*</sub><sup>′</sup> + *Δ**x*<sub>*k**a*</sub>
*y*<sub>*k**a*</sub> = *y*<sup>′</sup><sub>*k**a*</sub> + *Δ**y*<sub>*k**a*</sub>, *z*<sub>*k**a*</sub> = *z*<sup>′</sup><sub>*k**a*</sub> + *Δ**z*<sub>*k**a*</sub>, **e** = **e**<sup>′</sup> + **δ**, **q** = **q**<sup>′</sup> + *Δ***q**
where *L*<sub>*k**o*</sub><sup>′</sup> and *Δ**l*<sub>*k*</sub> are nominal values and zero errors of initial cable length, respectively. *L*<sub>*k**r*</sub><sup>′</sup> and *ψ*<sub>*k**r*</sub> are measurement values and measurement errors of the cables length, *x*<sup>′</sup> and *Δ**x* are the nominal values and errors of x coordinates of the end-effector, respectively. *e*<sub>*k**x*</sub> is the measurement noise in x direction of measurement sensors. *y*<sup>′</sup> and *Δ**y* are the nominal values and errors of y coordinates of the end-effector, respectively. *e*<sub>*k**y*</sub> is the measurement noise in y direction of measurement sensors. *z*<sup>′</sup> and *Δ**z* are the nominal values and errors of z coordinates of the end-effector, respectively. *e*<sub>*k**x*</sub> is the measurement noise in z direction of measurement sensors. *x*<sub>*k**a*</sub><sup>′</sup> and *Δ**x*<sub>*k**a*</sub> are the nominal values and errors of x coordinate of each cable outlet, respectively. *y*<sup>′</sup><sub>*k**a*</sub> and *Δ**y*<sub>*k**a*</sub> are the nominal values and errors of y coordinate of each cable outlet, respectively. *z*<sup>′</sup><sub>*k**a*</sub> and *Δ**z*<sub>*k**a*</sub> are the nominal values and errors of z coordinate of each cable outlet, respectively. **e**<sup>′</sup> and **δ** are the end-effector nominal position parameters set and the position parameter errors set, respectively. **q**<sup>′</sup> and *Δ***q** are nominal kinematic parameters set and kinematic errors set, respectively.

The error identification model is expressed as:
**δ** = *f*(*L*<sub>*k**r*</sub>,**q**<sup>′</sup>+*Δ***q**) − (**e**<sup>′</sup>+**e**<sub>*k*</sub>)
where *f*(*x*) is the forward kinematics equation of the CDPR for 3D printing.

Equation (20) can be rewritten as a functional form.
**e**<sub>*k*</sub> = *f*(*L*<sub>*k**r*</sub>, **q**<sup>′</sup>+*Δ***q**) − **e**<sup>′</sup> − **δ**

For the CDPR, the function equation based on the cable length residual is more convenient for the implementation of the nonlinear least square method. Hence, the functional equation for error identification is rewritten as follows:
$$\\begin{matrix}
{\\mathbf{\\zeta}\_{i} = {L\_{ko}}^{\\prime} + \\Delta l\_{k} + {L\_{kr}}^{\\prime}} \\\\
{- \\sqrt{(x^{\\prime} + \\Delta x - x\_{ka}^{\\prime} - \\Delta x\_{ka})^{2} + {(y^{\\prime} + \\Delta y - y\_{ka}^{\\prime} - \\Delta y\_{ka})}^{2} + {(z^{\\prime} + \\Delta z - z\_{ka}^{\\prime} - \\Delta z\_{ka})}^{2}}} \\\\
\\end{matrix}$$
where **ζ**<sub>*i*</sub> is a set of cable length residuals caused by the measurement noise of the measurement sensors.

The error identification problems are usually solved through the nonlinear least square method. Hence, the equation of error identification can be written as follows:
$$F = \\min\\limits\_{\\{{\\Delta l\_{k},(\\Delta x\_{ka},\\Delta y\_{ka},\\Delta z\_{ka})}\\}}{\\sum\\limits\_{i = 1}^{n}{{\\mathbf{\\zeta}\_{i}}^{T}\\mathbf{\\zeta}\_{i}}},k \\in \\{ 1,2,3\\}$$

## 5.2. Simulation Verification

The specific simulation calibration process is as follows:

\(1\) The optimal measurement positions set **N**<sub>*l*</sub>, is regarded as the measurement positions set. The solving of inverse kinematic equation is performed based on the nominal values of the measurement positions. The cable length **L**<sub>*k**r*, *l*</sub>, corresponding to the each measurement position can be obtained, which is used to simulate the cable length measured by motion capture system.

\(2\) By substituting the cable length **L**<sub>*k**r*, *l*</sub>, into the forward kinematic model based on the true values of geometric parameters, the actual coordinates **e**<sub>*l*</sub>, of each measurement position will be calculated. The true values of the geometric parameters contain the true coordinate values of the cable outlets and the true values of the initial cable length, which are given ideal parameter during the simulation. The main purpose of this step is to simulate the measurement value of the motion capture system in the actual calibration process.

\(3\) The position errors of end-effector **δ**<sub>*i*</sub>, are calculated and substituted into the error identification equation Equation (23). The error identification values *Δ***q**<sub>*i*</sub>, are calculated and compensated to each kinematic parameter **q**<sub>*i*</sub><sup>′</sup> = **q**<sub>*i*</sub><sup>′</sup> + *Δ***q**<sub>*i*</sub>, *i* = *i* + 1 and the new nominal value, **q**<sub>*i*</sub><sup>′</sup>, of each parameter is obtained.

\(4\) Compare the absolute value of *Δ***q**<sub>*i*</sub> with the given threshold *ε*, if *Δ***q**<sub>*i*</sub> ≤ *ε*, the simulation calibration is terminated, otherwise *i* = *i* + 1, return to step (1).

Because the motion control system is based on the equivalent model of the CDPR for 3D printing mentioned in <a href="#sec3-sensors-18-02898" data-ref-type="sec">Section 3</a>, the process of simulation calibration is based on the equivalent kinematics model. The true coordinates of the three virtual cable outlets are **a**<sub>1</sub> = (−260, −150.111, 78), **a**<sub>2</sub> = (260, −150.111, 78), and **a**<sub>3</sub> = (0, 300.222, 78), respectively, which are given ideal parameter during the simulation. The initial position of end-effector is specified as (0, 0, 330), the true values of the initial cable length can be determined using inverse kinematics as *L*<sub>*k**o*</sub> = 392 mm, (*k* = 1, 2, 3). Before the kinematic calibration of the CDPR for 3D printing, preliminary measurement should be performed to obtain the nominal values of the kinematic parameters. According to the existing measuring tools, the errors of the preliminary measurement are generally within 3 mm. In the simulation, random errors in the range of ±3 mm are added to the true values of geometric parameters to obtain the nominal values of kinematic parameters. Therefore, it is assumed that the nominal values of the initial cable length are *L*<sub>1*o*</sub><sup>′</sup> = 390 mm, *L*<sub>2*o*</sub><sup>′</sup> = 391.5 mm, and *L*<sub>3*o*</sub><sup>′</sup> = 389 mm, respectively. The nominal coordinates of each cable outlet are **a**<sub>1</sub><sup>′</sup> = (−258, −149, 79), **a**<sub>2</sub><sup>′</sup> = (263, −148, 77), and **a**<sub>3</sub><sup>′</sup> = (0, 301, 78.5), respectively.

Based on the data listed above, the threshold is set as *ε* = 1 × 10<sup>−4</sup> mm, the simulation calibration of a CDPR for 3D printing is performed. After the parameters iteration, the termination condition is satisfied while the parameter identification time is *i* = 4. The specific simulation results are shown in <a href="#sensors-18-02898-t002" data-ref-type="table">Table 2</a>.

As shown in <a href="#sensors-18-02898-f008" data-ref-type="fig">Figure 8</a>a, before the calibration, the maximum construction volume error in the optimal measurement positions set is *Δ**V*<sub>0max </sub> = 14.4223 mm. After four times error identification, the maximum structural volume error in the optimal measurement positions set is *Δ**V*<sub>4max </sub> = 5.4321 × 10<sup>−9</sup> mm.

The position errors in x direction are set as *Δ**x*<sub>*l*</sub>, the average absolute value of the position errors in *x* direction can be expressed as $\\Delta e\_{x} = {\\sum\\limits\_{l}^{4}{\|\\Delta x\_{l}}}\|/4$,which is used as a standard to study the position errors in *x* direction of the end-effector. Similarly, the position errors of *y* and *z* axis are measured as *Δ**e*<sub>*y*</sub> and *Δ**e*<sub>*z*</sub>. <a href="#sensors-18-02898-f008" data-ref-type="fig">Figure 8</a>b shows the variation of the *Δ**e*<sub>*x*</sub>, *Δ**e*<sub>*y*</sub>, and *Δ**e*<sub>*z*</sub> with the error identification times. It can be seen from <a href="#sensors-18-02898-f008" data-ref-type="fig">Figure 8</a>b that before calibration, the average absolute values of the position errors in *x*, *y*, and *z* directions are *Δ**e*<sub>*x*0</sub> = 1.1772 mm, *Δ**e*<sub>*y*0</sub> = 1.9062 mm, and *Δ**e*<sub>*z*0</sub> = 14.1970 mm, respectively. After four times error identification, the mean absolute value of the position errors in the direction of each coordinate axis are reduced to *Δ**e*<sub>*x*4</sub> = 8.4842 × 10<sup>−10</sup> mm, *Δ**e*<sub>*y*4</sub> = 1.2539 × 10<sup>−9</sup> mm, and *Δ**e*<sub>*z*4</sub> = 1.5540 × 10<sup>−9</sup> mm. The experimental results show that this calibration method is of fast convergence speed and favorable calibration accuracy.

# 6. Calibration Experiment

The experimental platform of the CDPR for 3D printing is shown in <a href="#sensors-18-02898-f009" data-ref-type="fig">Figure 9</a>, the stepping motor drives the linear module to drive the fixed-length cables to realize the movement of the end-effector. Therefore, the displacements of the sliders on the linear module is equal to the change of cable length. The coordinates of the markers attached to the sliders can be measured with the motion capture system.

## 6.1. Measurement

Before calibrating the coordinate values of each cable outlet and the initial cable length accurately, the initial measurement is carried out in order to obtain the reasonable structural parameters required by the control system. The upper surface of the base is the plane (*z* = 0 mm), the centroid of the regular triangular base is regarded as the origin of the global coordinate frame. The coordinate values of the cable outlets and the initial cable length measured in the global coordinate frame are shown in <a href="#sensors-18-02898-t003" data-ref-type="table">Table 3</a>.

As mentioned in <a href="#sec5dot2-sensors-18-02898" data-ref-type="sec">Section 5.2</a>, the coordinates of the cable outlets that should be calibrated are the coordinates of the three virtual cable outlets. According to the measured coordinates of each cable outlet, the coordinate values of virtual cable outlets are calculated based on the equivalent principle described in <a href="#sec3dot1-sensors-18-02898" data-ref-type="sec">Section 3.1</a>. The length of each cable group is the same as that of the corresponding virtual cables. The specific coordinate values of virtual cable outlets are shown in <a href="#sensors-18-02898-t004" data-ref-type="table">Table 4</a>.

Before the measurement, the markers should be adhere to the prototype to obtain the positions of each mobile component. The markers are distributed in three places, as shown in <a href="#sensors-18-02898-f010" data-ref-type="fig">Figure 10</a>. The first maker place is on the end-effector where the markers are triangle-distributed. The second maker place is triangle-distributed on the base. The centroid of the equilateral triangle is the coordinate origin of the global coordinate frame. The third maker place is located on the sliders of linear modules, the differences between the *z* values of the markers measured by the motion capture system and the *z* values of markers in the initial state are the cable length variables.

During the data measurement, the end-effector is moved to the above optimal positions (*z* = 110 mm), respectively. Meanwhile, the actual coordinate value of each position and the cable length are measured by the motion capture system synchronously.

## 6.2. Data Processing

In this calibration experiment, the measurement rate of the motion capture system is 60 frames per second. The effective test time for each location is 3 s. The coordinate of the markers is based on the world coordinate system determined by self-calibration of the motion capture system. However, the world coordinate system and the global coordinate system of the experimental device are difficult to be coincident though artificial setting. Therefore, in order to apply the measurement data to the error identification, the coordinate values measured by the motion capture system should be converted first.

The coordinate system A is set as the world coordinate system, the coordinate system B is set as the global system of experimental device, and the coordinate origin of coordinate system B in the coordinate system A is <sup>*A*</sup>**p**<sub>*B*0</sub>. Through determining <sup>*A*</sup>**p**<sub>*B*0</sub> and *y* axis direction vector of the coordinate system B in the coordinate system A, the conversion relationship between the two coordinate systems can be determined.<sup>*A*</sup>**p**<sub>*B*0</sub> is the centroid coordinate values of the regular triangle determined by the markers on the base, the *y* axis direction vector of the coordinate system B is also determined by the three attached markers on the base, which is the unit direction vector from a marker to the origin of coordinate system B. After obtaining the measurement data as described above, the rotation matrix <sub>*B*</sub><sup>*A*</sup>**R** between the coordinate system A and the coordinate system B can be calculated out. The coordinate values of the point *p* that is measured by the motion capture system in coordinate system A is <sup>*A*</sup>**p**, the position of point **p** in coordinate system B is as follows:
<sup>*B*</sup>**p** = <sub>*B*</sub><sup>*A*</sup>**R**<sup>−1</sup>(<sup>*A*</sup>**p**−<sup>*A*</sup>**p**<sub>*B*0</sub>)
where <sub>*B*</sub><sup>*A*</sup>**R**<sup>−1</sup> is the inverse matrix of rotation matrix <sub>*B*</sub><sup>*A*</sup>**R**.

After the data measurement, the coordinate values of three markers (F1, F2, F3) on the base in the coordinate system B are shown in <a href="#sensors-18-02898-t005" data-ref-type="table">Table 5</a>. According to the data, it is calculated that <sup>*A*</sup>**p**<sub>*B*0</sub> is (542.4536, 48.4085, 23.1651) and the specific value of rotation matrix <sub>*B*</sub><sup>*A*</sup>**R** is as follows:
$${}\_{B}^{A}\\mathbf{R} = \\begin{bmatrix}
0.483768011 & 0.875196271 & 0 \\\\
{- 0.875196271} & 0.483768011 & 0 \\\\
0 & 0 & 1 \\\\
\\end{bmatrix}$$

In the motion control that is based on the initial measured kinematic parameters, the coordinate values of each optimal position are measured and the measured data are transformed by the mentioned method. The structural volume errors and the position errors of each optimal position are obtained as shown in <a href="#sensors-18-02898-f011" data-ref-type="fig">Figure 11</a>. It can be seen from <a href="#sensors-18-02898-f011" data-ref-type="fig">Figure 11</a> that the error along the *z* axis is the main factor leading to the excessive position errors of the CDPR for 3D printing. Before calibration, the specific range of position errors and structural volume error of each optimal position are as shown in <a href="#sensors-18-02898-t006" data-ref-type="table">Table 6</a>.

The position errors of the end-effector reflects the accuracy of the CDPR for 3D printing. According to the analysis of the data in <a href="#sensors-18-02898-f011" data-ref-type="fig">Figure 11</a> and <a href="#sensors-18-02898-t006" data-ref-type="table">Table 6</a>, it can be seen that the position errors along *z* axis are more than 20 mm. The accuracy requirement for 3D printing can not be satisfied due to the position errors. In order to solve this problem, the error identification should be implemented based on the above measurement data. Through compensating the initial measured kinematic parameter with identification results, the printing accuracy will be improved.

In order to obtain valid error identification results, it is necessary to set a reasonable convergence threshold based on the measurement noise of the motion capture system. Through measuring the position coordinates of one fixed point repeatedly, the measurement noise of the motion capture system can be obtained. As shown in <a href="#sensors-18-02898-f012" data-ref-type="fig">Figure 12</a>, the fluctuation of the measurement noise is −0.12939 mm ≤*e*<sub>*k*</sub> ≤ 0.22407 mm. To reduce the influence of measurement noise on the error identification, a threshold as *ξ* = 0.80 mm is set. When the absolute value of the end-effector position errors meet the constraint \|*δ*\| ≤ *ξ*, the error identification cycle is terminated and the calibration is completed. After the parameter iteration, the position errors of the end-effector changing with the identification times are shown in <a href="#sensors-18-02898-f013" data-ref-type="fig">Figure 13</a>, after calibration, the errors of the end positions are as shown in <a href="#sensors-18-02898-t007" data-ref-type="table">Table 7</a>. The identification errors of kinematic parameters are shown in <a href="#sensors-18-02898-t008" data-ref-type="table">Table 8</a>.

From the above experimental results, one can obtain that the construction volume error of the end-effector position is reduced from 23.4662 mm to 0.4740 mm through the calibration. Among the position errors in each direction, the kinematic calibration is obviously effective to reduce the position errors along the *z* axis. The average errors in the z direction are reduced from −23.0636 mm to 0.3594 mm. Different from the position errors in the x and y directions, the position errors along the *z* axis will accumulate on the printed components as the printing layers increases for the fused deposition modeling. The proposed calibration method can bring obvious improvement of the position accuracy in the *z* direction and therefore, improve the printing accuracy of the CDPR for 3D printing effectively.

## 6.3. Calibration Result Verification

The calibrated kinematic parameters are applied to the control system, and the end-effector is controlled to move in four planes: Z = 175 mm, Z = 195 mm, Z = 215 mm, and Z=235 mm. On each plane, the ideal motion curve of the end-effector is the straight line from (20, 20) to (−20, −20). The static position errors of points on the line are measured to verify the effect of kinematic calibration. The measurement positions along the line are as follows: *M*<sub>0</sub>(−20, −20), *M*<sub>1</sub>(−16, −16), *M*<sub>2</sub>(−12, −12), *M*<sub>3</sub>(−8, −8), *M*<sub>4</sub>(−4, −4), *M*<sub>5</sub>(0, 0), *M*<sub>6</sub>(4, 4), *M*<sub>7</sub>(8, 8), *M*<sub>8</sub>(12, 12), *M*<sub>9</sub>(16, 16), and *M*<sub>10</sub>(20, 20). The static position errors measurement is carried out to eliminate the influence of delay of the controller, actuator, spring, etc. In the static case, the error of the end-effector mainly comes from the error of kinematic parameters.

One can obtain from <a href="#sensors-18-02898-f014" data-ref-type="fig">Figure 14</a> that the measured positions covers four planes with different heights and the position errors of the calibrated CDPR for 3D printing in each direction of the end-effector remains within −0.80 mm ≤ *δ* ≤ 0.80 mm. The measurement results indicate that the proposed calibration method is effective and verified for measurement positions outside optimal positions set.

# 7. Conclusions

This paper presents a CDPR for 3D printing, which can improve the workspace of 3D printing device at a lower manufacturing cost, due to the extension range and flexibility of cables.

In order to solve the kinematic calibration problem of CDPR for 3D printing, a kinematic calibration method based on cable length residuals is proposed. The functional formula is established between the measured information of the motion capture system and the actual kinematic parameters. According to the structure characteristics of the CDPR for 3D printing, a coordinate system conversion method for data measurement of the motion capture system is proposed, which provides the guarantee for obtaining accurate measurement data. The accuracy and effectiveness of this calibration method are verified though simulation. Furthermore, the kinematic calibration experiment is carried out on the basis of synchronous measurement of end-effector position errors and cable length variables with motion capture system.

In order to further improve the efficiency of calibration and measurement, an optimal selection scheme for measurement positions is proposed. The simulation results of error modeling and analysis indicate that the accuracy of CDPR for 3D printing increases with printing height, and the maximum error on each horizontal plane is distributed close to the boundary of the workspace. The calibration experiments are carried out based on the optimized positions set, and the construction volume error of the end-effector is reduced from 23.4805 mm to 0.6157 mm. In addition, the proposed calibration method is effective and verified for measurement positions outside optimal positions set through experiments.

---

## Author Contributions

S.Q. conceptualized and directed the research project. S.Q., K.B and B.Z. prepared the manuscript. S.O. and K.B. performed the experiments and data analysis with N.W. All authors discussed the results and commented on the manuscript.

## Funding

This research was funded by National Natural Science Foundation of China (Grant No. 51605126 and 51575150, 91748109) and Postdoctoral Science Foundation of China (Grant No. 2016M600479).

## Conflicts of Interest

The authors declare no conflict of interest.

---

# References


1. Berti, A.; Merlet, J.P.; Carricato, M. Solving the direct geometrico-static problem of underconstrained cable-driven parallel robots by interval analysis. *Int. J. Robot. Res.* 2016, 35, 723–739. DOI: 10.1177/0278364915595277.

2. Zi, B.; Qian, S. *Design, Analysis and Control of Cable-Suspended Parallel Robots and Its Applications*. Springer; Singapore; 2017.

3. Zi, B.; Li, Y. Conclusions in theory and practice for advancing the applications of cable-driven mechanisms. *Chin. J. Mech. Eng.* 2017, 30, 763–765. DOI: 10.1007/s10033-017-0148-7.

4. Zhou, B.; Zi, B.; Qian, S. Dynamics-based nonsingular interval model and luffing angular response field analysis of the dacs with narrowly bounded uncertainty. *Nonlinear Dyn.* 2017, 90, 2599–2626. DOI: 10.1007/s11071-017-3826-1.

5. Zi, B.; Sun, H.; Zhang, D. Design, analysis and control of a winding hybrid-driven cable parallel manipulator. *Robot. Comput. Integr. Manuf.* 2017, 48, 196–208. DOI: 10.1016/j.rcim.2017.04.002.

6. Barnett, E.; Gosselin, C. Large-scale 3d printing with a cable-suspended robot. *Addit. Manuf.* 2015, 7, 27–44. DOI: 10.1016/j.addma.2015.05.001.

7. Izard, J.B.; Dubor, A.; Hervé, P.E.; Cabay, E.; Culla, D.; Rodriguez, M.; Barrado, M. Large-scale 3d printing with cable-driven parallel robots. *Constr. Robot.* 2017, 1, 69–76. DOI: 10.1007/s41693-017-0008-0.

8. Qian, S.; Zi, B.; Shang, W.; Xu, Q. A review on cable-driven parallel robots. *Chin. J. Mech. Eng.* 2018, 31, 66. DOI: 10.1186/s10033-018-0267-9.

9. Gang, C.; Tong, L.; Ming, C.; Xuan, J.Q.; Xu, S.H. Review on kinematics calibration technology of serial robots. *Int. J. Precis. Eng. Manuf.* 2014, 15, 1759–1774. DOI: 10.1007/s12541-014-0528-1.

10. Zhuang, H.; Yan, J.; Masory, O. Calibration of stewart platforms and other parallel manipulators by minimizing inverse kinematic residuals. *J. Robot. Syst.* 1998, 15, 395–405. DOI: 10.1002/(SICI)1097-4563(199807)15:7<395::AID-ROB2>3.0.CO;2-H.

11. Dong, W.; Lin, W.; Qian, C.; Ye, C.; Gao, H. GA-based modified D-H method calibration modelling for 6-DoFs serial robot. In *Proceedings of the Youth Academic Annual Conference of Chinese Association of Automation (YAC)* Wuhan, China; 11–13 November 2016; pp. 225–230.

12. Du, G.; Shao, H.; Chen, Y.; Zhang, P.; Liu, X. An online method for serial robot self-calibration with CMAC and UKF. *Robot. Comput. Integr. Manuf.* 2016, 42, 39–48. DOI: 10.1016/j.rcim.2016.05.006.

13. Joubair, A.; Long, F.Z.; Bigras, P.; Bonev, I.A. Use of a force-torque sensor for self-calibration of a 6-DOF medical robot. *Sensors* 2016, 16. DOI: 10.3390/s16060798. PMID: 27258278.

14. Li, J.; Kaneko, A.M.; Endo, G.; Fukushima, E.F. In-field self-calibration of robotic manipulator using stereo camera: Application to humanitarian demining robot. *Adv. Robot.* 2015, 29, 1045–1059. DOI: 10.1080/01691864.2015.1012555.

15. Yin, S.; Ren, Y.; Zhu, J.; Yang, S.; Ye, S. A vision-based self-calibration method for robotic visual inspection systems. *Sensors* 2013, 13, 16565–16582. DOI: 10.3390/s131216565. PMID: 24300597.

16. Daney, D.; Andreff, N.; Chabert, G.; Papegay, Y. Interval method for calibration of parallel robots: Vision-based experiments. *Mech. Mach. Theor.* 2006, 41, 929–944. DOI: 10.1016/j.mechmachtheory.2006.03.014.

17. Majarena, A.C.; Santolaria, J.; Samper, D.; Aguilar, J.J. An overview of kinematic and calibration models using internal/external sensors or constraints to improve the behavior of spatial parallel mechanisms. *Sensors* 2010, 10, 10256–10297. DOI: 10.3390/s101110256. PMID: 22163469.

18. Joubair, A.; Slamani, M.; Bonev, I.A. Kinematic calibration of a five-bar planar parallel robot using all working modes. *Robot. Comput. Integr. Manuf.* 2013, 29, 15–25. DOI: 10.1016/j.rcim.2012.10.002.

19. Joubair, A.; Nubiola, A.; Bonev, I. Calibration efficiency analysis based on five observability indices and two calibration models for a six-axis industrial robot. *SAE Int. J. Aerosp.* 2013, 6, 161–168. DOI: 10.4271/2013-01-2117.

20. Joshi, S.A.; Surianarayan, A. Calibration of a 6-DOF cable robot using two inclinometers. *Perform. Metr. Intell. Syst.* 2003, 3660–3665.

21. Ren, X.D.; Feng, Z.R.; Su, C.P. A new calibration method for parallel kinematics machine tools using orientation constraint. *Int. J. Mach. Tools Manuf.* 2009, 49, 708–721. DOI: 10.1016/j.ijmachtools.2009.03.004.

22. Lau, D. Initial length and pose calibration for cable-driven parallel robots with relative length feedback. *Cable-Driven Parallel Robots*. Springer; Cham, Switzerland; 2018; pp. 140–151.

23. Duan, X.; Qiu, Y.; Duan, Q.; Du, J. Calibration and motion control of a cable-driven parallel manipulator based triple-level spatial positioner. *Adv. Mech. Eng.* 2014, 6, 368018. DOI: 10.1155/2014/368018.

24. Mustafa, S.K.; Yang, G.; Song, H.Y.; Lin, W.; Chen, I.M. Self-calibration of a biologically inspired 7 DOF cable-driven robotic arm. *IEEE/ASME Trans. Mechatron.* 2010, 13, 66–75. DOI: 10.1109/TMECH.2007.915024.

25. Borm, J.H.; Menq, C.H. Determination of optimal measurement configurations for robot calibration based on observability measure. *Int. J. Robot. Res.* 1991, 10, 51–63. DOI: 10.1177/027836499101000106.

26. Menq, C.H.; Borm, J.H.; Lai, J.Z. Identification and observability measure of a basis set of error parameters in robot calibration. *J. Mech. Transm. Autom. Des.* 1989, 111, 513–518. DOI: 10.1115/1.3259031.

27. Driels, M.R.; Pathre, U.S. Significance of observation strategy on the design of robot calibration experiments. *J. Field Robot.* 2010, 7, 197–223. DOI: 10.1002/rob.4620070206.

28. Nahvi, A.; Hollerbach, J.M.; Hayward, V. Calibration of a parallel robot using multiple kinematic closed loops. In *Proceedings of the IEEE International Conference on Robotics and Automation* San Diego, CA, USA; 8–13 May 1994; pp. 407–412.

29. Jia, Q.; Wang, S.; Chen, G.; Wang, L.; Sun, H. A novel optimal design of measurement configurations in robot calibration. *Math. Probl. Eng.* 2018, 4689710. DOI: 10.1155/2018/4689710.

30. Li, T.; Sun, K.; Jin, Y.; Liu, H. A novel optimal calibration algorithm on a dexterous 6 DOF serial robot-with the optimization of measurement poses number. In *Proceedings of the IEEE International Conference on Robotics and Automation* Shanghai, China; 9–13 May 2011; pp. 975–981.

31. Zhou, J.; Nguyen, H.N.; Kang, H.J. Selecting optimal measurement poses for kinematic calibration of industrial robots. *Adv. Mech. Eng.* 2014, 6, 291389. DOI: 10.1155/2014/291389.

32. Wang, D.; Wang, J.; Zhu, X.; Shao, Y. Determination of optimal measurement configurations for polishing robot calibration. *Int. J. Model. Identif. Control* 2014, 21, 211–222. DOI: 10.1504/IJMIC.2014.060014.

33. Daney, D.; Papegay, Y.; Madeline, B. Choosing measurement poses for robot calibration with the local convergence method and tabu search. *Int. J. Robot. Res.* 2005, 24, 501–518. DOI: 10.1177/0278364905053185.

34. Sun, Y.; Hollerbach, J.M. Active robot calibration algorithm, IEEE International Conference on Robotics and Automation. In *Proceedings of the IEEE International Conference on Robotics and Automation* Pasadena, CA, USA; 19–23 May 2008; pp. 1276–1281.

35. Nategh, M.J.; Agheli, M.M. A total solution to kinematic calibration of hexapod machine tools with a minimum number of measurement configurations and superior accuracies. *Int. J. Mach. Tools Manuf.* 2009, 49, 1155–1164. DOI: 10.1016/j.ijmachtools.2009.08.009.

36. Wang, H.; Gao, T.; Kinugawa, J.; Kosuge, K. Finding measurement configurations for accurate robot calibration: Validation with a cable-driven robot. *IEEE Trans. Robot.* 2017, 33, 1156–1169. DOI: 10.1109/TRO.2017.2707562.

37. Bo, O. Efficient computation method of force-closure workspace for 6-dof cable-driven parallel manipulators. *J. Mech. Eng.* 2013, 49, 34–41.

---

# Figures and Tables

<figure>
<img src="sensors-18-02898-g001.png" id="sensors-18-02898-f001" alt="Three-dimensional (3D) model of the cable-driven parallel robot (CDPR) for 3D Printing." />
<figcaption aria-hidden="true">Three-dimensional (3D) model of the cable-driven parallel robot (CDPR) for 3D Printing.</figcaption>
</figure>

<figure>
<img src="sensors-18-02898-g002.png" id="sensors-18-02898-f002" alt="Sketch of the CDPR for 3D Printing." />
<figcaption aria-hidden="true">Sketch of the CDPR for 3D Printing.</figcaption>
</figure>

<figure>
<img src="sensors-18-02898-g003.png" id="sensors-18-02898-f003" alt="Schematic diagram of the equivalent model." />
<figcaption aria-hidden="true">Schematic diagram of the equivalent model.</figcaption>
</figure>

<figure>
<img src="sensors-18-02898-g004.png" id="sensors-18-02898-f004" alt="Equivalent schematic diagram of the CDPR for 3D Printing." />
<figcaption aria-hidden="true">Equivalent schematic diagram of the CDPR for 3D Printing.</figcaption>
</figure>

<figure>
<img src="sensors-18-02898-g005.png" id="sensors-18-02898-f005" alt="The workspace of the CDPR for 3D Printing." />
<figcaption aria-hidden="true">The workspace of the CDPR for 3D Printing.</figcaption>
</figure>

<figure>
<img src="sensors-18-02898-g006.png" id="sensors-18-02898-f006" alt="The workspace of the CDPR for 3D Printing." />
<figcaption aria-hidden="true">The workspace of the CDPR for 3D Printing.</figcaption>
</figure>

<figure>
<img src="sensors-18-02898-g007.png" id="sensors-18-02898-f007" alt="The distribution of candidate points: (a) The planform of Z = 110 mm; (b) Discretized boundary of the Z = 110 mm." />
<figcaption aria-hidden="true">The distribution of candidate points: (<strong>a</strong>) The planform of Z = 110 mm; (<strong>b</strong>) Discretized boundary of the Z = 110 mm.</figcaption>
</figure>

<figure>
<img src="sensors-18-02898-g008.png" id="sensors-18-02898-f008" alt="The relationships between errors and identification times: (a) The change of construction volume errors; (b) The change of position errors." />
<figcaption aria-hidden="true">The relationships between errors and identification times: (<strong>a</strong>) The change of construction volume errors; (<strong>b</strong>) The change of position errors.</figcaption>
</figure>

<figure>
<img src="sensors-18-02898-g009.png" id="sensors-18-02898-f009" alt="Experimental device of kinematic calibration." />
<figcaption aria-hidden="true">Experimental device of kinematic calibration.</figcaption>
</figure>

<figure>
<img src="sensors-18-02898-g010.png" id="sensors-18-02898-f010" alt="The layout of the markers." />
<figcaption aria-hidden="true">The layout of the markers.</figcaption>
</figure>

<figure>
<img src="sensors-18-02898-g011.png" id="sensors-18-02898-f011" alt="The position errors of the each optimal position before calibration." />
<figcaption aria-hidden="true">The position errors of the each optimal position before calibration.</figcaption>
</figure>

<figure>
<img src="sensors-18-02898-g012.png" id="sensors-18-02898-f012" alt="Measurement noise of the motion capture system." />
<figcaption aria-hidden="true">Measurement noise of the motion capture system.</figcaption>
</figure>

<figure>
<img src="sensors-18-02898-g013.png" id="sensors-18-02898-f013" alt="The relations between position errors and identification times." />
<figcaption aria-hidden="true">The relations between position errors and identification times.</figcaption>
</figure>

<figure>
<img src="sensors-18-02898-g014.png" id="sensors-18-02898-f014" alt="The error distributions of the end-effector after calibration." />
<figcaption aria-hidden="true">The error distributions of the end-effector after calibration.</figcaption>
</figure>

<div id="sensors-18-02898-t001" class="table-wrap">

sensors-18-02898-t001_Table 1

<div class="caption">

###### 

Coordinates of the optimal measurement positions.

</div>

|                 | *x*/mm   | *y*/mm  | *z*/mm |
|-----------------|----------|---------|--------|
| *q*<sub>1</sub> | −30.2310 | 24.5370 | 110    |
| *q*<sub>2</sub> | −42.5000 | 24.5370 | 110    |
| *q*<sub>3</sub> | −31.8320 | 43.015  | 110    |
| *q*<sub>4</sub> | −36.3660 | 13.9120 | 110    |

</div>

<div id="sensors-18-02898-t002" class="table-wrap">

sensors-18-02898-t002_Table 2

<div class="caption">

###### 

The simulation results of the kinematic calibration.

</div>

<table>
<thead>
<tr class="header">
<th>Geometric<br />
Errors</th>
<th>Preliminary<br />
Errors (mm)</th>
<th>Error Identification Values (<em>i</em> = 1, 2, 3, 4)/(mm)</th>
<th></th>
<th></th>
<th></th>
</tr>
</thead>
<tbody>
<tr class="odd">
<td>Δ<em>x</em><sub>1<em>a</em></sub></td>
<td>2.0000</td>
<td>2.3485</td>
<td>−0.3485</td>
<td><span class="math inline">1.9536 × 10<sup>−8</sup></span></td>
<td><span class="math inline"> − 0.9257 × 10<sup>−8</sup></span></td>
</tr>
<tr class="even">
<td>Δ<em>y</em><sub>1<em>a</em></sub></td>
<td>−1.1110</td>
<td>−1.3170</td>
<td>0.2060</td>
<td><span class="math inline"> − 2.9026 × 10<sup>−9</sup></span></td>
<td><span class="math inline">3.0060 × 10<sup>−9</sup></span></td>
</tr>
<tr class="odd">
<td>Δ<em>z</em><sub>1<em>a</em></sub></td>
<td>−1.0000</td>
<td>−1.0225</td>
<td>0.0225</td>
<td><span class="math inline"> − 1.4291 × 10<sup>−9</sup></span></td>
<td><span class="math inline">9.5420 × 10<sup>−10</sup></span></td>
</tr>
<tr class="even">
<td>Δ<em>x</em><sub>2<em>a</em></sub></td>
<td>3.0000</td>
<td>3.2568</td>
<td>−0.2568</td>
<td><span class="math inline"> − 3.2210 × 10<sup>−10</sup></span></td>
<td><span class="math inline">0.0950 × 10<sup>−9</sup></span></td>
</tr>
<tr class="odd">
<td>Δ<em>y</em><sub>2<em>a</em></sub></td>
<td>−2.1110</td>
<td>−1.9264</td>
<td>−0.1845</td>
<td><span class="math inline">1.0411 × 10<sup>−10</sup></span></td>
<td><span class="math inline"> − 2.1103 × 10<sup>−10</sup></span></td>
</tr>
<tr class="even">
<td>Δ<em>z</em><sub>2<em>a</em></sub></td>
<td>1.0000</td>
<td>1.0648</td>
<td>−0.0648</td>
<td><span class="math inline">2.5231 × 10<sup>−7</sup></span></td>
<td><span class="math inline">1.4530 × 10<sup>−9</sup></span></td>
</tr>
<tr class="odd">
<td>Δ<em>x</em><sub>3<em>a</em></sub></td>
<td>0.0000</td>
<td>−0.1376</td>
<td>0.1376</td>
<td><span class="math inline">6.4737 × 10<sup>−8</sup></span></td>
<td><span class="math inline">1.9241 × 10<sup>−9</sup></span></td>
</tr>
<tr class="even">
<td>Δ<em>y</em><sub>3<em>a</em></sub></td>
<td>−0.7780</td>
<td>−1.9027</td>
<td>1.1245</td>
<td><span class="math inline">2.0310 × 10<sup>−4</sup></span></td>
<td><span class="math inline">8.4712 × 10<sup>−10</sup></span></td>
</tr>
<tr class="odd">
<td>Δ<em>z</em><sub>3<em>a</em></sub></td>
<td>−0.5000</td>
<td>−0.2944</td>
<td>0.2055</td>
<td><span class="math inline"> − 2.0476 × 10<sup>−5</sup></span></td>
<td><span class="math inline"> − 4.8903 × 10<sup>−7</sup></span></td>
</tr>
<tr class="even">
<td>Δ<em>l</em><sub>1</sub></td>
<td>2.0000</td>
<td>2.4042</td>
<td><span class="math inline">−</span> 0.4042</td>
<td><span class="math inline"> − 4.2842 × 10<sup>−7</sup></span></td>
<td><span class="math inline">2.2291 × 10<sup>−7</sup></span></td>
</tr>
<tr class="odd">
<td>Δ<em>l</em><sub>2</sub></td>
<td>1.0000</td>
<td>0.1776</td>
<td>0.3223</td>
<td><span class="math inline"> − 5.9930 × 10<sup>−10</sup></span></td>
<td><span class="math inline">6.5831 × 10<sup>−10</sup></span></td>
</tr>
<tr class="even">
<td>Δ<em>l</em><sub>3</sub></td>
<td>3.0000</td>
<td>1.8487</td>
<td>1.1509</td>
<td><span class="math inline">2.0394 × 10<sup>−4</sup></span></td>
<td><span class="math inline">1.5926 × 10<sup>−7</sup></span></td>
</tr>
</tbody>
</table>

</div>

<div id="sensors-18-02898-t003" class="table-wrap">

sensors-18-02898-t003_Table 3

<div class="caption">

###### 

The initial measurement of kinematic parameters.

</div>

| Kinematic Parameters | Parameters without Calibration (mm) |
|----------------------|-------------------------------------|
| A group cable length | 382.0                               |
| B group cable length | 387.0                               |
| C group cable length | 388.0                               |
| The cable outlet A1  | (306.0, −149.5, 77.0)               |
| The cable outlet A2  | (282.5, −188.5, 78.0)               |
| The cable outlet B1  | (−305.2, −149.1, 81.0)              |
| The cable outlet B2  | (−281.6, −189.7, 81.0)              |
| The cable outlet C1  | (−23.5, 339.0, 75.0)                |
| The cable outlet C2  | (23.5, 338.6, 77.0)                 |

</div>

<div id="sensors-18-02898-t004" class="table-wrap">

sensors-18-02898-t004_Table 4

<div class="caption">

###### 

The initial measurement of the virtual cable outlets.

</div>

| Virtual Cable Outlets | Parameters without Calibration (mm) |
|-----------------------|-------------------------------------|
| A                     | (253.90, −146.50, 77.5)             |
| B                     | (−253.30, −146.25, 81)              |
| C                     | (0, 292.50, 76)                     |

</div>

<div id="sensors-18-02898-t005" class="table-wrap">

sensors-18-02898-t005_Table 5

<div class="caption">

###### 

The coordinate values of the markers on the base.

</div>

| Markers on the Base | Coordinate Value (mm)          |
|---------------------|--------------------------------|
| F1                  | (547.7330, 160.4733, 24.0144)  |
| F2                  | (362.9547, −147.6336, 22.0150) |
| F3                  | (716.6375, −158.7557, 23.4959) |

</div>

<div id="sensors-18-02898-t006" class="table-wrap">

sensors-18-02898-t006_Table 6

<div class="caption">

###### 

The Error distribution area of the end-effector before calibration.

</div>

|               | *Δ***x** | *Δ***y** | *Δ***z** | *Δ***E** |
|---------------|----------|----------|----------|----------|
| Maximal value | −3.4567  | 2.5909   | −21.9312 | 22.3526  |
| Minimum value | −4.1500  | 1.5527   | −24.4203 | 24.8190  |
| Average value | −3.8023  | 2.0687   | −23.0636 | 23.4662  |

</div>

<div id="sensors-18-02898-t007" class="table-wrap">

sensors-18-02898-t007_Table 7

<div class="caption">

###### 

Error distribution area of the end-effector after calibration.

</div>

|               | *Δ***x** | *Δ***y** | *Δ***z** | *Δ***E** |
|---------------|----------|----------|----------|----------|
| Maximal value | 0.4251   | 0.2145   | 0.4146   | 0.6314   |
| Minimum value | −0.1354  | −0.4783  | 0.3427   | 0.6038   |
| Average value | 0.2674   | −0.1547  | 0.3594   | 0.4740   |

</div>

<div id="sensors-18-02898-t008" class="table-wrap">

sensors-18-02898-t008_Table 8

<div class="caption">

###### 

The experimental results of kinematic calibration.

</div>

<table>
<thead>
<tr class="header">
<th>Kinematic<br />
Parameters</th>
<th>Before<br />
Calibration (mm)</th>
<th>Error Identification Value (<em>l</em> = 1, 2)<br />
(mm)</th>
<th>After<br />
Calibration (mm)</th>
<th></th>
</tr>
</thead>
<tbody>
<tr class="odd">
<td><span class="math inline"><em>x</em><sub>1<em>a</em></sub></span></td>
<td>253.9000</td>
<td>2.07820</td>
<td>1.16230</td>
<td>257.1405</td>
</tr>
<tr class="even">
<td><span class="math inline"><em>y</em><sub>1<em>a</em></sub></span></td>
<td>−146.5000</td>
<td>−1.25331</td>
<td>−0.68765</td>
<td>−148.4410</td>
</tr>
<tr class="odd">
<td><span class="math inline"><em>z</em><sub>1<em>a</em></sub></span></td>
<td>77.5000</td>
<td>1.85782</td>
<td>−0.46523</td>
<td>78.8926</td>
</tr>
<tr class="even">
<td><span class="math inline"><em>x</em><sub>2<em>a</em></sub></span></td>
<td>−253.3000</td>
<td>−3.31530</td>
<td>−0.40253</td>
<td>−257.0180</td>
</tr>
<tr class="odd">
<td><span class="math inline"><em>y</em><sub>2<em>a</em></sub></span></td>
<td>−146.2500</td>
<td>−1.87986</td>
<td>−0.32561</td>
<td>−148.4550</td>
</tr>
<tr class="even">
<td><span class="math inline"><em>z</em><sub>2<em>a</em></sub></span></td>
<td>81.0000</td>
<td>−1.72365</td>
<td>−0.25632</td>
<td>79.0201</td>
</tr>
<tr class="odd">
<td><span class="math inline"><em>x</em><sub>3<em>a</em></sub></span></td>
<td>0.0000</td>
<td>−0.12250</td>
<td>0.12585</td>
<td>0.0033</td>
</tr>
<tr class="even">
<td><span class="math inline"><em>y</em><sub>3<em>a</em></sub></span></td>
<td>292.5000</td>
<td>5.71754</td>
<td>0.53260</td>
<td>298.7501</td>
</tr>
<tr class="odd">
<td><span class="math inline"><em>z</em><sub>3<em>a</em></sub></span></td>
<td>76.0000</td>
<td>4.66235</td>
<td>−2.18561</td>
<td>78.4767</td>
</tr>
<tr class="even">
<td><span class="math inline"><em>l</em><sub>1</sub></span></td>
<td>382.0000</td>
<td>1.60682</td>
<td>1.26320</td>
<td>384.8700</td>
</tr>
<tr class="odd">
<td><span class="math inline"><em>l</em><sub>2</sub></span></td>
<td>387.0000</td>
<td>−2.65896</td>
<td>0.54825</td>
<td>384.8893</td>
</tr>
<tr class="even">
<td><span class="math inline"><em>l</em><sub>3</sub></span></td>
<td>388.0000</td>
<td>−4.85620</td>
<td>2.15421</td>
<td>385.2980</td>
</tr>
</tbody>
</table>

</div>
