#include "OrientationInterpolator.h"

OrientationInterpolator::OrientationInterpolator()
{
    toolName = "TOOLPLATE";
    bTimeDep = false;
    qStart = new Eigen::Quaternion<double>;
    qTarget = new Eigen::Quaternion<double>;
    qOutput = new Eigen::Quaternion<double>;
    qDelta = new Eigen::Quaternion<double>;
    qLast = new Eigen::Quaternion<double>;


}

void OrientationInterpolator::SetRobot(pRobot thisRobot)
{
    if(thisRobot != NULL)
        mRobot = thisRobot;
}

void OrientationInterpolator::SetToolName(string tname)
{
    toolName = tname;
}

void OrientationInterpolator::Reset()
{
    int linkid;
    linkid = mRobot->GetLinkIndex(toolName);
    startOrientationAsMatrix3 = mRobot->GetReferenceFrame(linkid,0).GetOrient();


    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            startOrientationAsEigen(i,j) = startOrientationAsMatrix3(i,j);
        }
    }

    *qStart = startOrientationAsEigen;
    *qOutput = *qStart;
    ConvertOutput();

    currTime =0;

}

void OrientationInterpolator::SetInterpolationTime(REALTYPE interpT, REALTYPE dt)
{
    interpolationTime =interpT;
    dT = dt;
    bTimeDep =true;
}

void OrientationInterpolator::SetTargetOrientation(const Matrix3 &targetOrient)
{
    targetOrientationAsMatrix3 = targetOrient;

    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            targetOrientationAsEigen(i,j) = targetOrientationAsMatrix3(i,j);
        }
    }

    *qTarget = targetOrientationAsEigen;
}





void OrientationInterpolator::GetOutput(Matrix3 &outputRotMat)
{
    outputRotMat = outputOrientationAsMatrix3;
}

void OrientationInterpolator::GetOutput(Vector3 &outputAngleAxis)
{
    outputAngleAxis = outputOrientationAsMatrix3.GetExactRotationAxis();
}

void OrientationInterpolator::UpdateTimeDependent()
{
    double interp = currTime/interpolationTime;
    if(interp<=1)
        Update(interp);
    currTime += dT;
}

void OrientationInterpolator::Update(double interp)
{
    if(interp>1)
        return;


    *qOutput = qStart->slerp(interp,*qTarget);

    ConvertOutput();

}

void OrientationInterpolator::UpdateWithAngularVelocity(Vector3 omega, double dt)
{
    if(omega.Norm() < 0.0001)
        return;
    *qLast = *qOutput;
    double ha = omega.Norm()*0.5*dt;
    qDelta->w() = cos(ha);
    qDelta->x() = omega(0)*sin(ha)/omega.Norm();
    qDelta->y() = omega(1)*sin(ha)/omega.Norm();
    qDelta->z() = omega(2)*sin(ha)/omega.Norm();
    *qOutput = (*qDelta) * (*qLast);
    ConvertOutput();
}

void OrientationInterpolator::ConvertOutput()
{
    outputOrientationAsEigen = qOutput->toRotationMatrix();
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            outputOrientationAsMatrix3(i,j) = outputOrientationAsEigen(i,j);
        }
    }
}
