#include "clpwrapper.hpp"


LPSolver::LPSolver() {
    ClpSimplex model;

    // Create model
    model_ = new ClpSimplex();

    // -1 for maximization, 1 for minimization
    model_->setOptimizationDirection(1);
}

LPSolver::~LPSolver(){
    delete model_;
}

double LPSolver::solve(const Eigen::VectorXd &f,
                const Eigen::MatrixXd &A,
                const Eigen::VectorXd &lb,
                const Eigen::VectorXd &ub,
                const Eigen::VectorXd &vlb,
                const Eigen::VectorXd &vub,
                Eigen::VectorXd &x){
    CoinPackedMatrix Amat  = from_EigenMatrix(A);
    const double *rowlb = lb.data();
    const double *rowub = ub.data();
    const double *collb = vlb.data();
    const double *colub = vub.data();
    
    const double *obj = f.data();
    model_->loadProblem(Amat, collb, colub, obj, rowlb, rowub);

    // solve problem
    model_->primal();
    // model_->dual();

    // get pointer to solution
    int nc = model_->getNumCols();
    const double* solution = model_->getColSolution();
    x = Eigen::Map<const Eigen::VectorXd>(solution, nc);

    // wrap result
    double ret = model_->getObjValue();

    return ret;
}
double LPSolver::solve(const Eigen::VectorXd &f,
                const Eigen::MatrixXd &A,
                const Eigen::VectorXd &b,
                Eigen::VectorXd &x){
    // set problem
    CoinPackedMatrix Amat  = from_EigenMatrix(A);
    const double *rowub = b.data();
    const double *obj = f.data();
    model_->loadProblem(Amat, NULL, NULL, obj, NULL, rowub);

    // solve problem
    // model_->primal();
    model_->dual();

    // get pointer to solution
    int nc = model_->getNumCols();    
    x.resize(nc);
    const double* solution = model_->getColSolution();
    x = Eigen::Map<const Eigen::VectorXd>(solution, nc);

    return model_->getObjValue();
}

CoinPackedMatrix LPSolver::from_EigenMatrix(const Eigen::MatrixXd& emat){
    int numels=emat.rows()*emat.cols();
    int *rowIndices = (int*)malloc(numels * sizeof(int));
    int *colIndices = (int*)malloc(numels * sizeof(int));
    double *elements = (double*)malloc(numels * sizeof(double));
    if (rowIndices == NULL || 
        colIndices == NULL ||
        elements == NULL) {
        printf("Memory not allocated.\n");
        exit(0);
    }
    int i(0);
    for(int r(0); r<emat.rows(); ++r){
        for(int c(0); c<emat.cols(); ++c){            
            rowIndices[i] = r;
            colIndices[i] = c;
            elements[i] = emat(r,c);
            i++;
        }
    }  
    CoinPackedMatrix ret = CoinPackedMatrix(
        false, rowIndices, colIndices, elements, numels);
    free(rowIndices);
    free(colIndices);
    free(elements);
    return ret;
}