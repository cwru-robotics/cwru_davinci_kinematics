// davinci_Jacobian_test_main.cpp
// get random legal joint values, q_vec; compute fk1
// generate random perturbations, dq; compute fk2
// compute dp_fk = fk2_translational()-fk1_translational()
// compute Jacobian
// compute dp_J = J*dq
// compare dp_J to dp_fk
// do analogous comparison for dphi1, dphi2, dphi3

#include <cwru_davinci_kinematics/davinci_kinematics.h>
#include <tf/LinearMath/Quaternion.h>
#include <Eigen/Geometry> 

using namespace std;


//fnc to generate random joint perturbations in range set by eps
void gen_rand_joint_perturbations(Vectorq7x1 &dqvec) {
    double eps = 0.000001; // perturbation range
    dqvec(6)=0;
    for (int i=0;i<6;i++) {
        dqvec(i) = eps*(1.0 - 0.5*((double) rand())/((double) RAND_MAX));
        }
    //dqvec<<0,eps,0,0,0,0,0; //if desired, override to test one joint at a time
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "davinci_Jacobian_test_main");

    // this is what we are testing: jacobian in davinci_fwd_solver
    Davinci_fwd_solver davinci_fwd_solver; //instantiate a forward-kinematics solver    
    Eigen::MatrixXd Jacobian; 

    Eigen::Vector3d dp_fk,dp_J,dp_err_vec; 
    Eigen::Matrix3d R1,R2,dR;
    Eigen::Vector3d dphi_J,dphi_fk,dphi_err_vec;
    double p_err,phi_err;
    int ans;

    Vectorq7x1 q_vec1, q_vec2, dq_vec; 
    Eigen::VectorXd dq_vec6x1,dp6x1;
    for (int i=0;i<7;i++) q_vec1(i) = 0.0;
    //q_vec1 << 0, 0, 0, 0, 0, 0, 0;
    q_vec2=q_vec1;
    dq_vec = q_vec1;
    
    dq_vec6x1.resize(6,1); //6x1 perturbation vector to be used with Jacobian
    dp6x1.resize(6,1);
    Eigen::Affine3d affine_fk1, affine_fk2; //holders for FK computations

    for (int itries = 0; itries < 10000; itries++) {
        davinci_fwd_solver.gen_rand_legal_jnt_vals(q_vec1);
        cout << "using q_vec = " << q_vec1.transpose() << endl;     
        gen_rand_joint_perturbations(dq_vec);
        q_vec2 = q_vec1 + dq_vec;
        //strip off 7'th joint variable, gripper opening angle, to get 6x1 vector of joint perturbations
        for (int i=0;i<6;i++) dq_vec6x1(i) = dq_vec(i);

        //compute FK for two joint-space poses, q_vec1, q_vec2, that are
        // very close to each other (random perturbations)
        ROS_INFO("gripper tip frame from FK: "); //pose from first q_vec
        affine_fk1 = davinci_fwd_solver.fwd_kin_solve(q_vec1);
        affine_fk2 = davinci_fwd_solver.fwd_kin_solve(q_vec2);
        cout << "affine linear (R): " << endl; //orientation from first q_vec
        cout << affine_fk1.linear() << endl <<endl;
        cout << "origin: " << affine_fk1.translation().transpose() << endl <<endl;
        //compute numerical perturbation of output, based on two FK calls:
        dp_fk= affine_fk2.translation()-affine_fk1.translation();
        cout<<"tip displacement due to perturbation of angles, per FK:"<<endl;
        cout<<"dp_fk = "<<dp_fk.transpose()<<endl<<endl;
        //what rotation operator is required to change from orientation 1 to orientation 2?
        dR = affine_fk2.linear()*affine_fk1.linear().transpose();
        cout<<"Rotation operator for perturbation, per FK: "<<endl;
        cout<<dR<<endl;
        //for perturbations, can get dPhi 3x1 vector from components of dR operator
        dphi_fk(0) = -dR(1,2);
        dphi_fk(1) = dR(0,2);
        dphi_fk(2) = -dR(0,1);
        cout<<"extracted dPhi vector from rotation operator:"<<endl;
        cout<<"dphi_fk = "<<dphi_fk.transpose()<<endl;
        
        //here's a call to the function under test: computing the Jacobian about q_vec1
        Jacobian = davinci_fwd_solver.compute_jacobian(q_vec1); //compute the Jacobian
        cout<<"computed Jacobian: "<<endl;
        cout<<Jacobian<<endl<<endl;
        cout<<"perturbation of first 6 joints: "; //this is the joint-space perturbation being considered
        cout<<"dq_vec6x1 = "<<dq_vec6x1.transpose()<<endl;

        dp6x1 = Jacobian*dq_vec6x1; // here is the Jacobian-based approximation of incremental Cartesian pose change
        //first three elements are dx,dy,dz, and next 3 are dphix, dphiy, dphiz
        cout<<"6x1 perturbation of pose per Jacobian and joint perturbations:"<<endl;
        cout<<"dp6x1 = "<<dp6x1.transpose()<<endl;
        dp_J = dp6x1.block<3,1>(0,0); //strip off dx,dy,dz
        dp_err_vec = dp_fk - dp_J; //compare Jacobian dp(dq) to FK dp(dq)
        cout<<"comparison of endpoint displacements, FK vs Jacobian: "<<endl;
        cout<<"dp_err_vec = "<<dp_err_vec.transpose()<<endl;
        p_err = dp_err_vec.norm();
        cout<<"dp err norm = "<<p_err<<endl;
        //express this as displacement disagreement vs magnitude of displacement
        //e.g., percentage err (but expressed as a fraction, not percentage)
        cout<<"fractional error: "<<p_err/(dp_fk.norm())<<endl<<endl;
        
        //repeat for eval of angular Jacobian:
        dphi_J = dp6x1.block<3,1>(3,0); //(dphi_x, dphi_y, dphi_z)
        dphi_err_vec =  dphi_fk - dphi_J; //compare FK vs Jacobian computations
        cout<<"comparison of incremental rotation vectors, FK vs Jacobian: "<<endl;
        cout<<"dphi_err_vec = "<<dphi_err_vec.transpose()<<endl;
        phi_err = dphi_err_vec.norm();
        cout<<"dphi err norm = "<<phi_err<<endl;
        //express as a normalized error (scalar)
        cout<<"fractional error: "<<phi_err/(dphi_fk.norm())<<endl<<endl;        
        cout<<"enter 1: "; //pause here for user to view the results
        cin>>ans; //poor-man's breakpoint
    }
}
