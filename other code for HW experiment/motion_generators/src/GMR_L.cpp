/**************************************************************************
***    Copyright (c) 2011 S. Mohammad Khansari-Zadeh, LASA Lab, EPFL,   ***
***          CH-1015 Lausanne, Switzerland, http://lasa.epfl.ch         ***
***************************************************************************
*
* The program is free for non-commercial academic use, but WITHOUT ANY
* WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
* FOR A PARTICULAR PURPOSE.. Please contact the author if you are interested
* in using the software for commercial purposes. The software must not be
* modified or distributed without prior permission of the authors. Please
* acknowledge the authors in any academic publications that have made use
* of this code or part of it. Please use this BibTex reference:
*
*   S. M. Khansari Zadeh and A. Billard, "Imitation learning of Globally
*   Stable Non-Linear Point-to-Point Robot Motions using Nonlinear
*   Programming", in Proceeding of the 2010 IEEE/RSJ International
*   Conference on Intelligent Robots and Systems (IROS 2010), Taipei,
*   Taiwan, October 2010
*
* The author DOES NOT take any responsibility for this software.
*
* To get latest upadate of the software please visit:
*                          http://lasa.epfl.ch/khansari
*
* Please send your feedbacks or questions to:
*                           mohammad.khansari_at_epfl.ch
***************************************************************************/

#include "GMR_L.h"
#include <fstream>
#include "float.h"

#ifdef USE_MATHLIB_NAMESPACE
    using namespace MathLib;
#endif

bool GaussianMixture::loadParams(const char fileName[])
{
    // Load coefficient of a GMM from a file (stored by saveParams Method
    // or with matlab
    std::ifstream fich(fileName);
    if (!fich.is_open())
        return false;

    fich >> dim >> nState;
    dim /= 2;
    inComponents.resize(dim);
    outComponents.resize(dim);
    for (int i=0; i<dim; i++)
    {
        inComponents[i] = i;
        outComponents[i] = i + dim;
    }

    Priors.Resize(nState);
    for (int k = 0; k < nState; k++)
    {
        fich >> Priors[k];
    }
    Mu.Resize(nState,2*dim);
    for (int k = 0; k < nState; k++)
    {
        for (int j = 0;j<2*dim;j++)
            fich >> Mu(k,j);
    }
    if(Sigma!=NULL) delete [] Sigma;
    Sigma = new Matrix[nState];
    for (int k = 0; k < nState; k++)
    {
        Sigma[k] = Matrix(2*dim,2*dim);
        for (int i = 0; i < 2*dim; i++)
        {
            for (int j = 0;j<2*dim;j++)
                fich >> Sigma[k](i,j);
        }
    }

    //preparing some temporary variables that we will need during the regression
    Pxi.Resize(nState);
    subSigma = new Matrix[nState];
    for (int k = 0; k<nState; k++)
        subSigma[k] = Matrix(dim,dim);

    Mu.GetColumnSpace(outComponents,subMuOut);
    Mu.GetColumnSpace(inComponents,subMuIn);
    Mu_k.Resize(2*dim);
    Mu_x_k.Resize(dim);
    subSigmaOut.Resize(dim,dim);
    subSigmaIn.Resize(dim,dim);
    isubSigmaIn.Resize(dim,dim);
    x_tmp.Resize(dim);
    x_tmp2.Resize(dim);
    x_tmp3.Resize(dim);

    return true;
}




void GaussianMixture::doRegression(Vector & x, Vector & xd)
{
    norm_f = 0.0f;
    for(int k=0;k<nState;k++){
        p_i = Priors[k]*pdfState(x,k);
        Pxi[k] = p_i;
        norm_f += p_i;
    }
    Pxi /= norm_f;


    for(int k=0;k<nState;k++)
    {
        Sigma[k].GetMatrixSpace(inComponents,inComponents,subSigmaIn);
        subSigmaIn.InverseSymmetric(isubSigmaIn);
        Sigma[k].GetMatrixSpace(outComponents,inComponents,subSigmaOut);
        subSigmaOut.Mult(isubSigmaIn,subSigma[k]);
    }

    spxi  =0;
    for(int k=0;k<nState;k++)
    {
        //x_tmp = subMuOut.GetRow(k,x_tmp) + subSigma[k]*(x-subMuIn.GetRow(k));
        subMuOut.GetRow(k,x_tmp);
        subMuIn.GetRow(k,Mu_x_k);
        x.Sub(Mu_x_k,x_tmp2);
        subSigma[k].Mult(x_tmp2,x_tmp3);
        x_tmp += x_tmp3;

        tsum = spxi + Pxi[k];
        c1   = spxi / tsum;
        c2   = Pxi[k] / tsum;
        spxi  += Pxi[k];

        xd *= c1;
        x_tmp *= c2;
        xd.Add(x_tmp,xd);
    }
}


double GaussianMixture::pdfState(Vector &x,int & state)
{
    /* Compute the probability density function at vector x,
     (given along the dimensions Components), for a given state */

    Mu.GetRow(state,Mu_k);
    Mu_k.GetSubVector(inComponents,Mu_x_k);
    Sigma[state].GetMatrixSpace(inComponents,inComponents,subSigmaIn);

    subSigmaIn.InverseSymmetric(isubSigmaIn,&det_Sigma_k);
    if(subSigmaIn.IsInverseOk())
    {
        if(det_Sigma_k == 0)
        {
            std::cout << "Warning: null determinant" << std::endl;
            for(unsigned int k=0;k<dim;k++)
                subSigmaIn(k,k)=subSigmaIn(k,k)+1.0;
            subSigmaIn.Inverse(isubSigmaIn,&det_Sigma_k);
            std::cout << det_Sigma_k << std::endl;
        }
        x.Sub(Mu_x_k,x_tmp);
        isubSigmaIn.Mult(x_tmp,x_tmp2);
        prob = x_tmp * x_tmp2;
        prob = exp(-0.5*prob)/sqrt(pow(2*3.14159,dim)*(fabs(det_Sigma_k)+DBL_MIN));

        if (prob < DBL_MIN)
            return DBL_MIN;
        else
            return prob;
    }
    else
    {
        std::cout << "Error inverting Sigma" << std::endl;
        return 0;
    }
}



double GaussianMixture::pdfState(Vector Vin,IndicesVector Components,int state)
{
  /* Compute the probability density function at vector Vin,
     (given along the dimensions Components), for a given state */
  Vector Mu_s;
  Matrix sig_s;
  Matrix inv_sig_s;
  double det_sig;
  double p;
  Mu.GetRow(state).GetSubVector(Components,Mu_s);
  Sigma[state].GetMatrixSpace(Components,Components,sig_s);
  sig_s.InverseSymmetric(inv_sig_s,&det_sig);
  if(sig_s.IsInverseOk())
    {
      if(det_sig == 0)
    {
      std::cout << "Warning: null determinant" << std::endl;
      for(unsigned int k=0;k<dim;k++)
        sig_s(k,k)=sig_s(k,k)+1.0;
      sig_s.Inverse(inv_sig_s,&det_sig);
      std::cout << det_sig << std::endl;
    }
      p=(Vin-Mu_s) * ( inv_sig_s*(Vin-Mu_s));
      p=exp(-0.5*p)/sqrt(pow(2*3.14159,dim)*(fabs(det_sig)+DBL_MIN));
      if(p < DBL_MIN) return DBL_MIN;
      else return p;
    }
  else
    {
      std::cout << "Error inverting Sigma" << std::endl;
      return 0;
    }
}

Vector GaussianMixture::doRegression(Vector in)
{
    Vector Pxi(nState);
    Matrix * subSigma;
    Matrix subMuIn;
    Matrix subMuOut;
    double norm_f = 0.0f;
    for(int s=0;s<nState;s++){
        double p_i = Priors[s]*pdfState(in,inComponents,s);
        Pxi(s) = p_i;
        norm_f += p_i;
    }
    Pxi= Pxi*(1/norm_f);
    subSigma = new Matrix[nState];
    Mu.GetColumnSpace(outComponents,subMuOut);
    Mu.GetColumnSpace(inComponents,subMuIn);

    for(int s=0;s<nState;s++)
    {
        Matrix isubSigmaIn;
        Matrix subSigmaOut;
        Sigma[s].GetMatrixSpace(inComponents,inComponents,subSigmaOut);
        subSigmaOut.Inverse(isubSigmaIn);
        Sigma[s].GetMatrixSpace(outComponents,inComponents,subSigmaOut);
        subSigma[s] = subSigmaOut*isubSigmaIn;
    }


    Vector sv(dim,true);
    Vector sp(dim,true);


    double spxi  =0;
    double spxi2 =0;
    for(int s=0;s<nState;s++)
    {
        sp = subMuOut.GetRow(s);
        sp = sp + subSigma[s]*(in-subMuIn.GetRow(s));

        double tsum = spxi + Pxi(s);
        double c1   = spxi     / tsum;
        double c2   = Pxi(s) / tsum;

        sv = sv *c1 + sp * c2;

        spxi  += Pxi(s);
        spxi2 += Pxi(s)*Pxi(s);


    }

    delete [] subSigma;
    //delete [] subSigmaVar;
    return sv;
}
void GaussianMixture::debug(void)
{
  /*display on std output info about current parameters */
  Vector v;
  Matrix smat;
  std::cout << "Nb state : "<< this->nState <<std::endl;
  std::cout << "Dimensions : "<< this->dim  <<std::endl;
  std::cout << "Priors : ";
  for(int i = 0;i<nState;i++) {
    std::cout << Priors[i] <<" ";
  }
  std::cout << std::endl;
  std::cout << "Means :";
  Mu.Print();
  std::cout << "Covariance Matrices :";
  for(int i =0;i<nState;i++) {
    //double det;
    //Matrix inv;
    //sigma[i].GetMatrixSpace(v,v,inv);
    //inv.Print();
    //std::cout << det << std::endl;
    Sigma[i].Print();
  }
}
