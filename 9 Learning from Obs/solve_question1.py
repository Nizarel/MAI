import numpy as np
import pandas as pd
from sklearn.tree import DecisionTreeClassifier
from sklearn import tree
import matplotlib.pyplot as plt

# Dataset from Question 1
data = {
    'A1': [1, 1, 0, 1, 1],
    'A2': [0, 0, 1, 1, 1],
    'A3': [0, 1, 0, 1, 0],
    'Output': [0, 0, 0, 1, 1]
}

df = pd.DataFrame(data, index=['x1', 'x2', 'x3', 'x4', 'x5'])
print("=" * 60)
print("DATASET")
print("=" * 60)
print(df)
print()

# Extract features and target
X = df[['A1', 'A2', 'A3']].values
y = df['Output'].values

# ============= ENTROPY CALCULATION =============
def calculate_entropy(y):
    """Calculate entropy of labels"""
    unique, counts = np.unique(y, return_counts=True)
    probabilities = counts / len(y)
    entropy = -np.sum([p * np.log2(p) for p in probabilities if p > 0])
    return entropy

# Calculate initial entropy
initial_entropy = calculate_entropy(y)
print("=" * 60)
print("ENTROPY CALCULATION")
print("=" * 60)

# Count class distribution
unique, counts = np.unique(y, return_counts=True)
print(f"Class distribution: {dict(zip(unique, counts))}")
print(f"  - Class 0: {counts[0]} examples")
print(f"  - Class 1: {counts[1]} examples")
print()

p_positive = counts[1] / len(y)
p_negative = counts[0] / len(y)

print(f"Initial Entropy H(S) = -({p_negative:.1f} * log2({p_negative:.1f}) + {p_positive:.1f} * log2({p_positive:.1f}))")
print(f"Initial Entropy H(S) = {initial_entropy:.4f}")
print()

# ============= INFORMATION GAIN CALCULATION =============
def calculate_information_gain(X, y, attribute_idx):
    """Calculate information gain for splitting on an attribute"""
    initial_entropy = calculate_entropy(y)
    
    # Split on attribute
    attribute_values = X[:, attribute_idx]
    unique_values = np.unique(attribute_values)
    
    weighted_entropy = 0
    splits = {}
    
    for value in unique_values:
        mask = attribute_values == value
        subset_y = y[mask]
        splits[value] = subset_y
        
        if len(subset_y) > 0:
            entropy = calculate_entropy(subset_y)
            weight = len(subset_y) / len(y)
            weighted_entropy += weight * entropy
    
    ig = initial_entropy - weighted_entropy
    
    return ig, splits

print("=" * 60)
print("INFORMATION GAIN FOR EACH ATTRIBUTE")
print("=" * 60)

attribute_names = ['A1', 'A2', 'A3']
ig_scores = {}

for idx, attr_name in enumerate(attribute_names):
    ig, splits = calculate_information_gain(X, y, idx)
    ig_scores[attr_name] = ig
    
    print(f"\nAttribute {attr_name}:")
    print(f"  Split on {attr_name}:")
    
    for value, subset_y in splits.items():
        count_0 = np.sum(subset_y == 0)
        count_1 = np.sum(subset_y == 1)
        subset_entropy = calculate_entropy(subset_y)
        print(f"    {attr_name}={value}: {len(subset_y)} examples [0:{count_0}, 1:{count_1}], H={subset_entropy:.4f}")
    
    weighted_entropy = sum((len(subset_y) / len(y)) * calculate_entropy(subset_y) 
                          for subset_y in splits.values())
    ig = initial_entropy - weighted_entropy
    
    print(f"  Weighted Entropy = {weighted_entropy:.4f}")
    print(f"  Information Gain IG({attr_name}) = {initial_entropy:.4f} - {weighted_entropy:.4f} = {ig:.4f}")

print()
print("=" * 60)
print("BEST SPLIT")
print("=" * 60)
best_attr = max(ig_scores, key=ig_scores.get)
print(f"Best splitting attribute: {best_attr} with IG = {ig_scores[best_attr]:.4f}")
print()

# ============= BUILD DECISION TREE =============
print("=" * 60)
print("DECISION TREE STRUCTURE")
print("=" * 60)

# Train decision tree (let sklearn build it automatically)
clf = DecisionTreeClassifier(random_state=42)
clf.fit(X, y)

# Export text representation
text_rep = tree.export_text(clf, feature_names=attribute_names)
print(text_rep)
print()

# ============= VISUALIZE DECISION TREE =============
print("=" * 60)
print("VISUALIZING DECISION TREE")
print("=" * 60)

fig, ax = plt.subplots(figsize=(20, 10))
tree.plot_tree(clf, 
               feature_names=attribute_names, 
               class_names=['Class 0', 'Class 1'],
               filled=True,
               ax=ax,
               fontsize=10,
               rounded=True)

output_path = r"c:\Users\nizare\Documents\GitHub\MAI\9 Learning from Obs\decision_tree_Q1.png"
plt.tight_layout()
plt.savefig(output_path, dpi=150, bbox_inches='tight')
print(f"Decision tree saved to: {output_path}")
plt.close()

print("\n" + "=" * 60)
print("SUMMARY")
print("=" * 60)
print(f"Initial Entropy: {initial_entropy:.4f}")
print(f"Information Gains: {ig_scores}")
print(f"Best First Split: {best_attr}")
print("=" * 60)
