/**
 * @file primitive_ros_utils.h
 * @brief Interface between primitive classes and ROS
 */
#ifndef MPL_PRIMITIVE_ROS_UTILS_H
#define MPL_PRIMITIVE_ROS_UTILS_H

// #include <mpl_basis/trajectory.h>
#include <common/common.h>
#include <copter/trajectory.h>

#include <planning_ros_msgs/PrimitiveArray.h>
#include <planning_ros_msgs/Trajectory.h>

/// Primitive2d to primitive ROS message
inline planning_ros_msgs::Primitive toPrimitiveROSMsg(const Primitive2d &pr, double z = 0)
{
    const auto cx = pr.getPr(0).coeff();
    const auto cy = pr.getPr(1).coeff();
    const auto cyaw = pr.getPrYaw().coeff();
    Vec6f cz = Vec6f::Zero();
    cz[5] = z;
    planning_ros_msgs::Primitive msg;
    msg.cx.resize(6);
    msg.cy.resize(6);
    msg.cz.resize(6);
    msg.cyaw.resize(6);
    for (int i = 0; i < 6; i++)
    {
        msg.cx[i] = cx(i);
        msg.cy[i] = cy(i);
        msg.cz[i] = cz(i);
        msg.cyaw[i] = cyaw(i);
    }
    msg.t = pr.getDT();
    msg.wt = pr.getWorkTime();
    return msg;
}

/// Primitive3d to primitive ROS message
inline planning_ros_msgs::Primitive toPrimitiveROSMsg(const Primitive3d &pr)
{
    const auto cx = pr.getPr(0).coeff();
    const auto cy = pr.getPr(1).coeff();
    const auto cz = pr.getPr(2).coeff();
    const auto cyaw = pr.getPrYaw().coeff();
    planning_ros_msgs::Primitive msg;
    msg.cx.resize(6);
    msg.cy.resize(6);
    msg.cz.resize(6);
    msg.cyaw.resize(6);
    for (int i = 0; i < 6; i++)
    {
        msg.cx[i] = cx(i);
        msg.cy[i] = cy(i);
        msg.cz[i] = cz(i);
        msg.cyaw[i] = cyaw(i);
    }
    msg.t = pr.getDT();
    msg.wt = pr.getWorkTime();
    return msg;
}

/// Multiple Primitive2d to Primitive array ROS message
inline planning_ros_msgs::PrimitiveArray toPrimitiveArrayROSMsg( const vec_E<Primitive2d> &prs, double z = 0 )
{
    planning_ros_msgs::PrimitiveArray msg;
    for (const auto &pr : prs)
        msg.primitives.push_back(toPrimitiveROSMsg(pr, z));
    return msg;
}

/// Multiple Primitive3d to Primitive array ROS message
inline planning_ros_msgs::PrimitiveArray toPrimitiveArrayROSMsg( const vec_E<Primitive3d> &prs )
{
    planning_ros_msgs::PrimitiveArray msg;
    for (const auto &pr : prs)
        msg.primitives.push_back(toPrimitiveROSMsg(pr));
    return msg;
}

/// Trajectory2d class to trajectory ROS message
inline planning_ros_msgs::Trajectory toTrajectoryROSMsg( const Trajectory2d &traj, double z = 0 )
{
    planning_ros_msgs::Trajectory msg;
    for (const auto &seg : traj.getPrimitives() )
        msg.primitives.push_back(toPrimitiveROSMsg(seg, z));

    // if ( traj.lambda().exist() )
    // {
    //     auto l = traj.lambda();
    //     msg.lambda.resize(l.segs.size());
    //     for (int i = 0; i < (int)l.segs.size(); i++)
    //     {
    //         msg.lambda[i].dT = l.segs[i].dT;
    //         msg.lambda[i].ti = l.segs[i].ti;
    //         msg.lambda[i].tf = l.segs[i].tf;
    //         msg.lambda[i].ca.resize(4);
    //         for (int j = 0; j < 4; j++) msg.lambda[i].ca[j] = l.segs[i].a(j);
    //     }
    // }
    return msg;
}

/// Trajectory3d class to trajectory ROS message
inline planning_ros_msgs::Trajectory toTrajectoryROSMsg( const Trajectory3d &traj )
{
    planning_ros_msgs::Trajectory msg;
    for (const auto &seg : traj.getPrimitives() )
        msg.primitives.push_back(toPrimitiveROSMsg(seg));

    // if (traj.lambda().exist()) {
    //     auto l = traj.lambda();
    //     msg.lambda.resize(l.segs.size());
    //     for (int i = 0; i < (int)l.segs.size(); i++) {
    //         msg.lambda[i].dT = l.segs[i].dT;
    //         msg.lambda[i].ti = l.segs[i].ti;
    //         msg.lambda[i].tf = l.segs[i].tf;
    //         msg.lambda[i].ca.resize(4);
    //         for (int j = 0; j < 4; j++) msg.lambda[i].ca[j] = l.segs[i].a(j);
    //     }
    // }

    return msg;
}

/// ROS message to Primitive2d class
inline Primitive2d toPrimitive2d(const planning_ros_msgs::Primitive &pr)
{
    Vec6f cx, cy, cyaw;
    for (int i = 0; i < 6; i++)
    {
        cx(i) = pr.cx[i];
        cy(i) = pr.cy[i];
        cyaw(i) = pr.cyaw[i];
    }
    vec_E<Vec6f> cs;
    cs.push_back(cx);
    cs.push_back(cy);
    cs.push_back(cyaw);

    return Primitive2d( cs, ControlMode::SNP, true, pr.t, pr.wt );
}

/// ROS message to Primitive3d class
inline Primitive3d toPrimitive3d(const planning_ros_msgs::Primitive &pr)
{
    Vec6f cx, cy, cz, cyaw;
    for (int i = 0; i < 6; i++)
    {
        cx(i) = pr.cx[i];
        cy(i) = pr.cy[i];
        cz(i) = pr.cz[i];
        cyaw(i) = pr.cyaw[i];
    }
    vec_E<Vec6f> cs;
    cs.push_back(cx);
    cs.push_back(cy);
    cs.push_back(cz);
    cs.push_back(cyaw);

    return Primitive3d( cs, ControlMode::SNP, true, pr.t, pr.wt );
}

/// ROS message to Trajectory2d class
inline Trajectory2d toTrajectory2d( const planning_ros_msgs::Trajectory &traj_msg ) 
{
    vec_E<Primitive2d> prs;
    for (const auto &it : traj_msg.primitives ) 
    {
        prs.push_back( toPrimitive2d(it) );
    }
    Trajectory2d traj(prs);
    return traj;

    // Constructor from ros msg
    // Trajectory2d traj;
    // traj.taus.push_back(0);
    // for (const auto &it : traj_msg.primitives ) 
    // {
    //     traj.segs.push_back(toPrimitive2d(it));
    //     traj.taus.push_back(traj.taus.back() + it.t);
    // }
    // traj.total_t_ = traj.taus.back();

    // if (!traj_msg.lambda.empty()) {
    //     Lambda l;
    //     for (int i = 0; i < (int)traj_msg.lambda.size(); i++) {
    //         LambdaSeg seg;
    //         seg.a(0) = traj_msg.lambda[i].ca[0];
    //         seg.a(1) = traj_msg.lambda[i].ca[1];
    //         seg.a(2) = traj_msg.lambda[i].ca[2];
    //         seg.a(3) = traj_msg.lambda[i].ca[3];
    //         seg.ti = traj_msg.lambda[i].ti;
    //         seg.tf = traj_msg.lambda[i].tf;
    //         seg.dT = traj_msg.lambda[i].dT;
    //         l.segs.push_back(seg);
    //         traj.total_t_ += seg.dT;
    //     }
    //     traj.lambda_ = l;
    //     std::vector<decimal_t> ts;
    //     for (const auto &tau : traj.taus) ts.push_back(traj.lambda_.getT(tau));
    //     traj.Ts = ts;
    // } else
    //     traj.total_t_ = traj.taus.back();
    // return traj;
}

/// ROS message to Trajectory3d class
inline Trajectory3d toTrajectory3d( const planning_ros_msgs::Trajectory &traj_msg )
{
    vec_E<Primitive3d> prs;
    for (const auto &it : traj_msg.primitives ) 
    {
        prs.push_back( toPrimitive3d(it) );
    }
    Trajectory3d traj(prs);
    return traj;

    // Trajectory3d traj;
    // traj.taus.push_back(0);
    // for (const auto &it : traj_msg.primitives) 
    // {
    //     traj.segs.push_back(toPrimitive3d(it));
    //     traj.taus.push_back(traj.taus.back() + it.t);
    // }

    // if (!traj_msg.lambda.empty()) {
    //     Lambda l;
    //     for (int i = 0; i < (int)traj_msg.lambda.size(); i++) {
    //         LambdaSeg seg;
    //         seg.a(0) = traj_msg.lambda[i].ca[0];
    //         seg.a(1) = traj_msg.lambda[i].ca[1];
    //         seg.a(2) = traj_msg.lambda[i].ca[2];
    //         seg.a(3) = traj_msg.lambda[i].ca[3];
    //         seg.ti = traj_msg.lambda[i].ti;
    //         seg.tf = traj_msg.lambda[i].tf;
    //         seg.dT = traj_msg.lambda[i].dT;
    //         l.segs.push_back(seg);
    //         traj.total_t_ += seg.dT;
    //     }
    //     traj.lambda_ = l;
    //     std::vector<decimal_t> ts;
    //     for (const auto &tau : traj.taus) ts.push_back(traj.lambda_.getT(tau));
    //     traj.Ts = ts;
    // } else
    //     traj.total_t_ = traj.taus.back();

    // traj.total_t_ = traj.taus.back();
    // return traj;
}

#endif
