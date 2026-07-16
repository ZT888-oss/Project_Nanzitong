import numpy as np
import matplotlib.pyplot as plt
import util

def discrimAnalysis(x, y):
    """
    Estimate the parameters in LDA/QDA and visualize the LDA/QDA models
    
    Inputs
    ------
    x: a N-by-2 2D array contains the 2D features of the N samples
    
    y: a N-by-1 1D array contains the labels of the N samples 
    
    Outputs
    -----
    A tuple of five elments: mu_1,mu_2,cov,cov_1,cov_2
    in which mu_1, mu_2 are mean vectors (as 1D arrays)
             cov, cov_1, cov_2 are covariance matrices (as 2D arrays)
    Besides producing the five outputs, you need also to plot 1 figure for LDA 
    and 1 figure for QDA in this function         
    """
    ### TODO: Write your code here
    # Separate classes
    x1 = x[y == 1]
    x2 = x[y == 2]

    N = x.shape[0]
    N1 = x1.shape[0]
    N2 = x2.shape[0]

    # ===== Means =====
    mu_1 = np.mean(x1, axis=0)
    mu_2 = np.mean(x2, axis=0)

    # ===== Covariances =====
    # QDA covariances
    cov_1 = np.dot((x1 - mu_1).T, (x1 - mu_1)) / N1
    cov_2 = np.dot((x2 - mu_2).T, (x2 - mu_2)) / N2

    # LDA shared covariance
    cov = (N1 * cov_1 + N2 * cov_2) / N

    # ===== Create grid =====
    x1_range = np.linspace(-4, 6, 100) #100 samples, there distance of eaach pint in range is (6+4)/(100 - 99) 
    x2_range = np.linspace(-5, 5, 100)
    X1, X2 = np.meshgrid(x1_range, x2_range)    #pair each X1 and X2 point in their range
    grid = np.c_[X1.ravel(), X2.ravel()]    #flaten 2D array of X1 and X2 into 1D by ravel() the stack falttened X1 and X2 into coordinates pair

    # ===== Gaussian density function =====
    def gaussian(x, mu, cov):
        d = x.shape[1]
        #Σ^-1  and |Σ|
        inv = np.linalg.inv(cov) 
        det = np.linalg.det(cov)
        norm_const = 1 / (np.sqrt((2 * np.pi)**d * det))
        diff = x - mu
        return norm_const * np.exp(-0.5 * np.sum(diff @ inv * diff, axis=1)) #diff @ inv * diff, axis=1):(x−μ)^T Σ^−1 (x−μ): xTAx=∑​(Ax)j​⋅xj​

    #LDA
    plt.figure()

    # scatter
    plt.scatter(x1[:, 0], x1[:, 1], c='blue')
    plt.scatter(x2[:, 0], x2[:, 1], c='red')

    # densities
    Z1 = gaussian(grid, mu_1, cov).reshape(X1.shape)
    Z2 = gaussian(grid, mu_2, cov).reshape(X1.shape)

    plt.contour(X1, X2, Z1, colors='blue')
    plt.contour(X1, X2, Z2, colors='red')

    # decision boundary (LDA)
    inv_cov = np.linalg.inv(cov)
    w = inv_cov @ (mu_1 - mu_2)
    b = -0.5 * mu_1.T @ inv_cov @ mu_1 + 0.5 * mu_2.T @ inv_cov @ mu_2

    Z = (grid @ w + b).reshape(X1.shape)
    plt.contour(X1, X2, Z, levels=[0], colors='black')

    plt.title("LDA")
    plt.xlim([-4, 6])
    plt.ylim([-5, 5])

    #QDA
    plt.figure()

    # scatter
    plt.scatter(x1[:, 0], x1[:, 1], c='blue')
    plt.scatter(x2[:, 0], x2[:, 1], c='red')

    # densities
    Z1 = gaussian(grid, mu_1, cov_1).reshape(X1.shape)
    Z2 = gaussian(grid, mu_2, cov_2).reshape(X1.shape)

    plt.contour(X1, X2, Z1, colors='blue')
    plt.contour(X1, X2, Z2, colors='red')

    # decision boundary (QDA)
    inv1 = np.linalg.inv(cov_1) #np.linalg: toolbox for martix math, tells that apply a linear algebra operatio
    inv2 = np.linalg.inv(cov_2)

    term1 = np.sum((grid - mu_1) @ inv1 * (grid - mu_1), axis=1) #@ repsenet matrix multiplication
    term2 = np.sum((grid - mu_2) @ inv2 * (grid - mu_2), axis=1)

    Z = (term1 - term2 + np.log(np.linalg.det(cov_1) / np.linalg.det(cov_2)))
    Z = Z.reshape(X1.shape)
    
    plt.contour(X1, X2, Z, levels=[0], colors='black')

    plt.title("QDA")
    plt.xlim([-4, 6])
    plt.ylim([-5, 5])

    plt.show()

    return (mu_1, mu_2, cov, cov_1, cov_2)
    
    #return (mu_male,mu_female,cov,cov_male,cov_female)
    

def misRate(mu_male,mu_female,cov,cov_male,cov_female,x,y):
    """
    Use LDA/QDA on the testing set and compute the misclassification rate
    
    Inputs
    ------
    mu_1,mu_2,cov,cov_1,mu_2: parameters from discrimAnalysis
    
    x: a N-by-2 2D array contains the 2D features of the N samples 
    
    y: a N-by-1 1D array contains the labels of the N samples 
    
    Outputs
    -----
    A tuple of two elements: (mis rate in LDA, mis rate in QDA )
    """
    ### TODO: Write your code here
    N = x.shape[0]

    # ======================
    # ======== LDA =========
    # ======================
    inv_cov = np.linalg.inv(cov)

    w = inv_cov @ (mu_male - mu_female)
    b = -0.5 * mu_male.T @ inv_cov @ mu_male + 0.5 * mu_female.T @ inv_cov @ mu_female

    scores = x @ w + b
    y_pred_lda = np.where(scores > 0, 1, 2)

    mis_lda = np.mean(y_pred_lda != y)

    # ======================
    # ======== QDA =========
    # ======================
    inv1 = np.linalg.inv(cov_male)
    inv2 = np.linalg.inv(cov_female)

    det1 = np.linalg.det(cov_male)
    det2 = np.linalg.det(cov_female)

    diff1 = x - mu_male
    diff2 = x - mu_female

    term1 = np.sum((diff1 @ inv1) * diff1, axis=1)
    term2 = np.sum((diff2 @ inv2) * diff2, axis=1)

    g1 = -0.5 * np.log(det1) - 0.5 * term1
    g2 = -0.5 * np.log(det2) - 0.5 * term2

    y_pred_qda = np.where(g1 > g2, 1, 2)

    mis_qda = np.mean(y_pred_qda != y)
    
    print(mis_lda, mis_qda)
    
    return (mis_lda, mis_qda)


if __name__ == '__main__':
    
    # load training data and testing data
    x_train, y_train = util.get_data_in_file('trainData.txt')
    x_test, y_test = util.get_data_in_file('testData.txt')
    
    # parameter estimation and visualization in LDA/QDA
    mu_male,mu_female,cov,cov_male,cov_female = discrimAnalysis(x_train,y_train)
    
    # misclassification rate computation
    mis_LDA,mis_QDA = misRate(mu_male,mu_female,cov,cov_male,cov_female,x_test,y_test)
    

    
    
    

    
