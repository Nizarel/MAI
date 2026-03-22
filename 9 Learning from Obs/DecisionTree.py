from matplotlib import pyplot as plt
from sklearn import datasets
from sklearn.tree import DecisionTreeClassifier 
from sklearn import tree

# Prepare the data data
iris = datasets.load_iris()
X = iris.data
y = iris.target

# Fit the classifier with max_depth=3
clf = DecisionTreeClassifier(max_depth=3, random_state=1234)
clf.fit(X, y)

# get the text representation
text_representation = tree.export_text(clf)
print(text_representation)

# or plot the result
tree.plot_tree(clf)
