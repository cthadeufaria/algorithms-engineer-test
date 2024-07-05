import numpy as np
from sklearn.naive_bayes import GaussianNB
from sklearn.preprocessing import MinMaxScaler
from algorithm.utils import compute_centroids


class Classifier:
    def __init__(self, limbs):
        self.min_distances = dict()
        self.positions = dict()
        self.labeled_data = limbs
        self.train()
        self.centroids = dict()
        self.class_probabilities = dict()

    def train(self):
        self.prepare_data()
        self.nb_classifier = GaussianNB()
        self.nb_classifier.fit(self.X_train, self.y_train)

    def prepare_data(self):
        X_train = list()
        y_train = list()
        for data in self.labeled_data:
            X_train.append(data.movement)
            y_train.append([str(data.position) for _ in range(len(data.movement))])

        self.X_train = self.normalize(np.array([item for sublist in X_train for item in sublist]))
        self.y_train = [item for sublist in y_train for item in sublist]

    def classify(self, time, kalman_filter):
        data = dict()
        for sensor in kalman_filter.keys():
            data[sensor] = self.normalize([
                np.array([t, x, y, z]) for t, x, y, z in zip(
                    time,
                    kalman_filter[sensor][0].coordinates, 
                    kalman_filter[sensor][1].coordinates,
                    kalman_filter[sensor][2].coordinates
                )
            ])
            self.centroids[sensor] = compute_centroids(data[sensor])
            self.class_probabilities[sensor] = self.nb_classifier.predict_proba(self.centroids[sensor].reshape(1, -1))

    def find_closest_class(self, unlabeled_centroids, class_centroids):

        for sensor, centroid in unlabeled_centroids.items():
            min_distance = float('inf')

            for limb in class_centroids:
                distance = np.linalg.norm(centroid - limb.centroid)
                if distance < min_distance:
                    min_distance = distance
                    self.min_distances[sensor] = min_distance
                    self.positions[sensor] = limb.position
        
        self.converge()

    def normalize(self, array: np.array):
        scaler = MinMaxScaler()
        return scaler.fit_transform(array)

    def converge(self):
        pass