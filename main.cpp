#include <iostream>
#include "CorrelativeMatch.hpp"
float sigma=0.1;
float map_resolution=0.05;
Eigen::MatrixXf generateGuassKernal(const int &sizes)
{
  if((sizes%2)!=1)
  {
    cout<<"sizes必须是一个奇数"<<endl;
    assert((sizes%2)==1);
  }
  Eigen::MatrixXf K;
  K.resize(sizes,sizes);
  for(int i=0;i<(sizes/2+1);i++)
  {
    for(int j=i;j<(sizes/2+1);j++)
    {
      Eigen::Vector2f temp;
      temp<<(sizes/2-i)*map_resolution,(sizes/2-j)*map_resolution;
      K(i,j)=1/((2*3.141692f)*pow(sigma,2))*exp(-temp.squaredNorm()/(2*pow(sigma,2)));
      K(i,sizes-j-1)=K(i,j);
      if(i!=j)
      {
      K(j,i)=K(i,j);
      K(j,sizes-i-1)=K(j,i);
      }
      if((sizes-i-1)!=i)
      {
      K.block(sizes-i-1,0,1,sizes)=K.block(i,0,1,sizes);
      }
    }
  }
  return K;
}

int main(int argc, char **argv) {
  
  Eigen::MatrixXf a=generateGuassKernal(9);
  cout<<a<<endl;
    std::cout << "Hello, world!" << std::endl;
    return 0;
}
