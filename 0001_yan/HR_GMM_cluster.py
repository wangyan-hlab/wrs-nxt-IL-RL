import numpy as np
from sklearn.cluster import KMeans
from scipy.stats import multivariate_normal as mvn

class GMM:
    """ Gaussian Mixture Model

    Parameters
    -----------
        k: int , number of gaussian distributions

        seed: int, will be randomly set if None

        max_iter: int, number of iterations to run algorithm, default: 200

    Attributes
    -----------
       centroids: array, k, number_features

       cluster_labels: label for each data point

    """

    def __init__(self, C, n_runs):
        self.C = C  # number of Guassians/clusters
        self.n_runs = n_runs

    def get_params(self):
        return (self.mu, self.pi, self.sigma)

    def calculate_mean_covariance(self, X, prediction):
        """Calculate means and covariance of different
            clusters from k-means prediction

        Parameters:
        ------------
        prediction: cluster labels from k-means

        X: N*d numpy array data points

        Returns:
        -------------
        intial_means: for E-step of EM algorithm

        intial_cov: for E-step of EM algorithm

        """
        d = X.shape[1]
        labels = np.unique(prediction)
        self.initial_means = np.zeros((self.C, d))
        self.initial_cov = np.zeros((self.C, d, d))
        self.initial_pi = np.zeros(self.C)

        counter = 0
        for label in labels:
            ids = np.where(prediction == label)  # returns indices
            self.initial_pi[counter] = len(ids[0]) / X.shape[0]
            self.initial_means[counter, :] = np.mean(X[ids], axis=0)
            de_meaned = X[ids] - self.initial_means[counter, :]
            Nk = X[ids].shape[0]  # number of data points in current gaussian
            self.initial_cov[counter, :, :] = np.dot(self.initial_pi[counter] * de_meaned.T, de_meaned) / Nk
            counter += 1
        assert np.sum(self.initial_pi) == 1

        return (self.initial_means, self.initial_cov, self.initial_pi)

    def _initialise_parameters(self, X):
        """Implement k-means to find starting
            parameter values.
            https://datascience.stackexchange.com/questions/11487/how-do-i-obtain-the-weight-and-variance-of-a-k-means-cluster
        Parameters:
        ------------
        X: numpy array of data points

        Returns:
        ----------
        tuple containing initial means and covariance

        _initial_means: numpy array: (C*d)

        _initial_cov: numpy array: (C,d*d)


        """
        n_clusters = self.C
        kmeans = KMeans(n_clusters=n_clusters, init="k-means++", max_iter=500, algorithm='auto')
        fitted = kmeans.fit(X)
        prediction = kmeans.predict(X)
        self._initial_means, self._initial_cov, self._initial_pi = self.calculate_mean_covariance(X, prediction)

        return (self._initial_means, self._initial_cov, self._initial_pi)

    def _e_step(self, X, pi, mu, sigma):
        """Performs E-step on GMM model
        Parameters:
        ------------
        X: (N x d), data points, m: no of features
        pi: (C), weights of mixture components
        mu: (C x d), mixture component means
        sigma: (C x d x d), mixture component covariance matrices
        Returns:
        ----------
        gamma: (N x C), probabilities of clusters for objects
        """
        N = X.shape[0]
        self.gamma = np.zeros((N, self.C))

        const_c = np.zeros(self.C)

        self.mu = self.mu if self._initial_means is None else self._initial_means
        self.pi = self.pi if self._initial_pi is None else self._initial_pi
        self.sigma = self.sigma if self._initial_cov is None else self._initial_cov

        for c in range(self.C):
            # Posterior Distribution using Bayes Rule
            self.gamma[:, c] = self.pi[c] * mvn.pdf(X, self.mu[c, :], self.sigma[c])

        # normalize across columns to make a valid probability
        gamma_norm = np.sum(self.gamma, axis=1)[:, np.newaxis]
        self.gamma /= gamma_norm

        return self.gamma

    def _m_step(self, X, gamma):
        """Performs M-step of the GMM
        We need to update our priors, our means
        and our covariance matrix.
        Parameters:
        -----------
        X: (N x d), data
        gamma: (N x C), posterior distribution of lower bound
        Returns:
        ---------
        pi: (C)
        mu: (C x d)
        sigma: (C x d x d)
        """
        N = X.shape[0]  # number of objects
        C = self.gamma.shape[1]  # number of clusters
        d = X.shape[1]  # dimension of each object

        # responsibilities for each gaussian
        self.pi = np.mean(self.gamma, axis=0)

        self.mu = np.dot(self.gamma.T, X) / np.sum(self.gamma, axis=0)[:, np.newaxis]

        for c in range(C):
            x = X - self.mu[c, :]  # (N x d)

            gamma_diag = np.diag(self.gamma[:, c])
            # x_mu = np.matrix(x)
            # gamma_diag = np.matrix(gamma_diag)
            sigma_c = np.matmul(np.matmul(x.T, gamma_diag), x)
            self.sigma[c, :, :] = (sigma_c) / np.sum(self.gamma, axis=0)[:, np.newaxis][c]

        return self.pi, self.mu, self.sigma

    def _compute_loss_function(self, X, pi, mu, sigma):
        """Computes lower bound loss function

        Parameters:
        -----------
        X: (N x d), data

        Returns:
        ---------
        pi: (C)
        mu: (C x d)
        sigma: (C x d x d)
        """
        N = X.shape[0]
        C = self.gamma.shape[1]
        self.loss = np.zeros((N, C))

        for c in range(C):
            dist = mvn(self.mu[c], self.sigma[c], allow_singular=True)
            self.loss[:, c] = self.gamma[:, c] * (
                        np.log(self.pi[c] + 0.00001) + dist.logpdf(X) - np.log(self.gamma[:, c] + 0.000001))
        self.loss = np.sum(self.loss)
        return self.loss

    def fit(self, X):
        """Compute the E-step and M-step and
            Calculates the lowerbound

        Parameters:
        -----------
        X: (N x d), data

        Returns:
        ----------
        instance of GMM

        """

        d = X.shape[1]
        self.mu, self.sigma, self.pi = self._initialise_parameters(X)

        try:
            for run in range(self.n_runs):
                self.gamma = self._e_step(X, self.mu, self.pi, self.sigma)
                self.pi, self.mu, self.sigma = self._m_step(X, self.gamma)
                loss = self._compute_loss_function(X, self.pi, self.mu, self.sigma)

                if run % 10 == 0:
                    print("Iteration: %d Loss: %0.6f" % (run, float(loss)))


        except Exception as e:
            print(e)

        return self

    def predict(self, X):
        """Returns predicted labels using Bayes Rule to
        Calculate the posterior distribution

        Parameters:
        -------------
        X: ?*d numpy array

        Returns:
        ----------
        labels: predicted cluster based on
        highest responsibility gamma.

        """
        labels = np.zeros((X.shape[0], self.C))

        for c in range(self.C):
            labels[:, c] = self.pi[c] * mvn.pdf(X, self.mu[c, :], self.sigma[c])
        labels = labels.argmax(1)
        return labels

    def predict_proba(self, X):
        """Returns predicted labels

        Parameters:
        -------------
        X: N*d numpy array

        Returns:
        ----------
        labels: predicted cluster based on
        highest responsibility gamma.

        """
        post_proba = np.zeros((X.shape[0], self.C))

        for c in range(self.C):
            # Posterior Distribution using Bayes Rule, try and vectorise
            post_proba[:, c] = self.pi[c] * mvn.pdf(X, self.mu[c, :], self.sigma[c])

        return post_proba

if __name__ == '__main__':
    # determine proper n_cluster for low and high level GMMs
    import os
    import matplotlib.pyplot as plt
    # plt.style.use("seaborn")
    from scipy.stats import multivariate_normal
    from sklearn.mixture import GaussianMixture

    this_dir, this_filename = os.path.split(__file__)
    ## Low level policy
    f = open(os.path.join(this_dir, "document", "Data_low.txt"), "r")
    fl = f.readlines()[2945*0:2945*100]  # [0:294500:50]
    S = []
    A = []
    n = 0
    for string in fl:
        n += 1
        strlist = string.replace("(", "").replace(")", "")
        data = eval(strlist)
        state = []

        for i in range(6):
            state.append(data[i])
        for j in range(18, len(data)):
            state.append(data[j])

        S.append(state)
        action = []
        # for k in range(6, 12):
        for k in range(12, 18):
            action.append(data[k])
        A.append(action)
        # print(n)
    STATE = np.asarray(S)
    ACTION = np.asarray(A)
    rdc_dim = 2
    u1, s1, v1 = np.linalg.svd(np.cov(np.transpose(ACTION)))
    u1_reduce = u1[:, :rdc_dim]
    ACTION_rdc = np.transpose(np.matmul(np.transpose(u1_reduce), np.transpose(ACTION)))

    lowest_bic = np.infty
    bic = []
    n_components_range = range(2,15)
    for n_components in n_components_range:
        gmm = GaussianMixture(n_components=n_components).fit(ACTION)  # Instantiate and fit the model
        print('Converged:', gmm.converged_) # Check if the model has converged
        bic.append(gmm.bic(ACTION))
        if bic[-1] < lowest_bic:
            print('n_components =', n_components, 'bic =', bic[-1],)
            lowest_bic = bic[-1]
            best_gmm = gmm
    print(bic, np.gradient(bic))
    plt.figure(1)
    plt.subplot(121)
    plt.title('BIC low level')
    plt.xlabel('n_components')
    plt.ylabel('BIC')
    plt.plot(range(2, 15), bic)
    plt.grid()
    plt.subplot(122)
    plt.title('BIC gradient low level')
    plt.xlabel('n_components')
    plt.ylabel('grad(BIC)')
    plt.plot(range(2, 15), np.gradient(bic))
    plt.grid()
    plt.show()
    # GMM = best_gmm
    # means = GMM.means_
    # covariances = GMM.covariances_
    # weights = GMM.weights_
    # for m, c, w in zip(means, covariances, weights):
    #     print(m, c, w)

    # gmm = GMM(C=3, n_runs=100)
    # gmm.fit(ACTION_rdc)
    # mu, pi, sigma = gmm.get_params()
    # print('mu =', mu)
    # print('pi =', pi)
    # print('sigma =', sigma)

    # x, y = np.meshgrid(np.sort(ACTION_rdc[:, 0]), np.sort(ACTION_rdc[:, 1]))
    # XY = np.array([x.flatten(), y.flatten()]).T
    # fig = plt.figure(figsize=(10, 10))
    # plt.axis([-360, -60, 350, 480])
    # plt.scatter(ACTION_rdc[:, 0], ACTION_rdc[:, 1], c='g', s=3, alpha=0.3)
    # for m, c in zip(means, covariances):
    #     print('m =', m)
    #     print('c =', c)
    #     multi_normal = multivariate_normal(mean=m, cov=c)
    #     plt.contour(np.sort(ACTION_rdc[:, 0]), np.sort(ACTION_rdc[:, 1]),
    #                 multi_normal.pdf(XY).reshape(len(ACTION_rdc), len(ACTION_rdc)), colors='red', alpha=0.7)
    #     plt.scatter(m[0], m[1], c='black', zorder=10, s=100)
    # plt.show()

    ## High level policy
    # f = open(os.path.join(this_dir, "document", "Data_high.txt"), "r")
    # fh = f.readlines()[11180*0:11180*100]  #[0:1118000:400]
    # S_H = []
    # S_L = []
    # n = 0
    # for string in fh:
    #     n += 1
    #     strlist = string.replace("(", "").replace(")", "")
    #     data = eval(strlist)
    #     state_now = []
    #     state_h = []
    #     for i in range(6):
    #         state_h.append(data[i])
    #     for j in range(12,len(data)):
    #         state_h.append(data[j])
    #     # print(state)
    #     S_H.append(state_h)
    #     state_l = []
    #     for k in range(6,12):
    #         state_l.append(data[k])
    #     S_L.append(state_l)
    #     # print(n)
    # STATE_H = np.asarray(S_H)
    # STATE_L = np.asarray(S_L)
    # rdc_dim = 2
    # u4, s4, v4 = np.linalg.svd(np.cov(np.transpose(STATE_L)))
    # u4_reduce = u4[:, :rdc_dim]
    # STATE_L_rdc = np.transpose(np.matmul(np.transpose(u4_reduce), np.transpose(STATE_L)))
    #
    # lowest_bic = np.infty
    # bic = []
    # n_components_range = range(2, 15)
    # for n_components in n_components_range:
    #     gmm = GaussianMixture(n_components=n_components).fit(STATE_L)  # Instantiate and fit the model
    #     print('Converged:', gmm.converged_)  # Check if the model has converged
    #     bic.append(gmm.bic(STATE_L))
    #     if bic[-1] < lowest_bic:
    #         print('n_components =', n_components, 'bic =', bic[-1], )
    #         lowest_bic = bic[-1]
    #         best_gmm = gmm
    # print(bic, np.gradient(bic))
    # plt.figure(2)
    # plt.subplot(121)
    # plt.title('BIC high level')
    # plt.xlabel('n_components')
    # plt.ylabel('BIC')
    # plt.plot(range(2, 15), bic)
    # plt.grid()
    # plt.subplot(122)
    # plt.title('BIC gradient high level')
    # plt.xlabel('n_components')
    # plt.ylabel('grad(BIC)')
    # plt.plot(range(2, 15), np.gradient(bic))
    # plt.grid()
    # plt.show()
    # # GMM = best_gmm
    # # means = GMM.means_
    # # covariances = GMM.covariances_
    # # weights = GMM.weights_
    # # for m, c, w in zip(means, covariances, weights):
    # #     print(m, c, w)