import numpy as np
import pandas as pd

# Dataset from Question 1
data = {
    'A1': [1, 1, 0, 1, 1],
    'A2': [0, 0, 1, 1, 1],
    'A3': [0, 1, 0, 1, 0],
    'Output': [0, 0, 0, 1, 1]
}

df = pd.DataFrame(data, index=['x1', 'x2', 'x3', 'x4', 'x5'])
X = df[['A1', 'A2', 'A3']].values
y = df['Output'].values

def calculate_entropy(y):
    """Calculate entropy of labels"""
    if len(y) == 0:
        return 0
    unique, counts = np.unique(y, return_counts=True)
    probabilities = counts / len(y)
    entropy = -np.sum([p * np.log2(p) for p in probabilities if p > 0])
    return entropy

def calculate_ig(X_subset, y_subset, attr_idx):
    """Calculate information gain for a specific attribute on a subset"""
    initial_entropy = calculate_entropy(y_subset)
    
    attr_values = X_subset[:, attr_idx]
    unique_vals = np.unique(attr_values)
    
    weighted_entropy = 0
    splits = {}
    
    for value in unique_vals:
        mask = attr_values == value
        subset_y = y_subset[mask]
        splits[value] = subset_y
        
        if len(subset_y) > 0:
            entropy = calculate_entropy(subset_y)
            weight = len(subset_y) / len(y_subset)
            weighted_entropy += weight * entropy
    
    ig = initial_entropy - weighted_entropy
    return ig, splits

print("=" * 80)
print("QUESTION 5: What is the attribute should be used in the first split?")
print("=" * 80)
print()

# From previous calculations
ig_a1 = 0.1710
ig_a2 = 0.4200
ig_a3 = 0.0200

print(f"Information Gains at Root:")
print(f"  IG(A1) = {ig_a1:.4f}")
print(f"  IG(A2) = {ig_a2:.4f} ← HIGHEST")
print(f"  IG(A3) = {ig_a3:.4f}")
print()
print(f"ANSWER TO QUESTION 5: A2")
print()

# ============= SECOND SPLIT ANALYSIS =============
print("=" * 80)
print("ANALYZING SECOND SPLIT")
print("=" * 80)
print()

# After splitting on A2:
print("After A2 split:")
print()

# A2=0 branch
mask_a2_0 = X[:, 1] == 0  # A2=0
X_a2_0 = X[mask_a2_0]
y_a2_0 = y[mask_a2_0]
indices_a2_0 = np.where(mask_a2_0)[0]

print(f"A2 = 0 branch:")
print(f"  Examples: x{indices_a2_0[0]+1}, x{indices_a2_0[1]+1}")
print(f"  Classes: {list(y_a2_0)}")
print(f"  Status: PURE (all Class 0) - NO FURTHER SPLIT NEEDED")
print()

# A2=1 branch (needs further splitting)
mask_a2_1 = X[:, 1] == 1  # A2=1
X_a2_1 = X[mask_a2_1]
y_a2_1 = y[mask_a2_1]
indices_a2_1 = np.where(mask_a2_1)[0]

print(f"A2 = 1 branch:")
print(f"  Examples: x{indices_a2_1[0]+1}, x{indices_a2_1[1]+1}, x{indices_a2_1[2]+1}")
print(f"  Classes: {list(y_a2_1)}")
print(f"  Entropy: {calculate_entropy(y_a2_1):.4f}")
print(f"  Status: NEEDS FURTHER SPLIT")
print()

print("=" * 80)
print("QUESTION 6: What is the attribute should be used in the second split?")
print("=" * 80)
print()

print(f"On the A2=1 branch, we need to split further.")
print(f"Calculating information gain for remaining attributes on A2=1 subset:")
print()

# Calculate IG for A1 and A3 on the A2=1 subset
ig_a1_on_a2_1, splits_a1 = calculate_ig(X_a2_1, y_a2_1, 0)  # A1
ig_a3_on_a2_1, splits_a3 = calculate_ig(X_a2_1, y_a2_1, 2)  # A3

print(f"Splitting on A1 (on A2=1 subset):")
for value in sorted(splits_a1.keys()):
    subset_y = splits_a1[value]
    subset_indices = indices_a2_1[X_a2_1[:, 0] == value]
    count_0 = np.sum(subset_y == 0)
    count_1 = np.sum(subset_y == 1)
    entropy = calculate_entropy(subset_y)
    print(f"  A1={int(value)}: x{subset_indices[0]+1} → Classes {list(subset_y)}, H={entropy:.4f}")

weighted_entropy_a1 = sum((len(subset_y)/len(y_a2_1)) * calculate_entropy(subset_y) 
                          for subset_y in splits_a1.values())
print(f"  Weighted Entropy = {weighted_entropy_a1:.4f}")
print(f"  IG(A1) = {calculate_entropy(y_a2_1):.4f} - {weighted_entropy_a1:.4f} = {ig_a1_on_a2_1:.4f}")
print()

print(f"Splitting on A3 (on A2=1 subset):")
for value in sorted(splits_a3.keys()):
    subset_y = splits_a3[value]
    subset_indices = indices_a2_1[X_a2_1[:, 2] == value]
    count_0 = np.sum(subset_y == 0)
    count_1 = np.sum(subset_y == 1)
    entropy = calculate_entropy(subset_y)
    if len(subset_indices) > 0:
        print(f"  A3={int(value)}: x{subset_indices[0]+1} → Classes {list(subset_y)}, H={entropy:.4f}")

weighted_entropy_a3 = sum((len(subset_y)/len(y_a2_1)) * calculate_entropy(subset_y) 
                          for subset_y in splits_a3.values())
print(f"  Weighted Entropy = {weighted_entropy_a3:.4f}")
print(f"  IG(A3) = {calculate_entropy(y_a2_1):.4f} - {weighted_entropy_a3:.4f} = {ig_a3_on_a2_1:.4f}")
print()

# Determine best second split
if ig_a1_on_a2_1 > ig_a3_on_a2_1:
    best_second = "A1"
    print(f"ANSWER TO QUESTION 6: A1 (IG = {ig_a1_on_a2_1:.4f})")
elif ig_a3_on_a2_1 > ig_a1_on_a2_1:
    best_second = "A3"
    print(f"ANSWER TO QUESTION 6: A3 (IG = {ig_a3_on_a2_1:.4f})")
else:
    best_second = "A1"  # Tie-breaker: choose A1
    print(f"ANSWER TO QUESTION 6: A1 (Tie with A3, choosing A1 by convention)")

print()

# ============= QUESTION 7: FINAL TREE =============
print("=" * 80)
print("QUESTION 7: Final Decision Tree")
print("=" * 80)
print()

tree_ascii = """
                        A2
                       /  \\
                    0 /    \\ 1
                     /      \\
                   0         A1
                  (y=0)      /  \\
                          0 /    \\ 1
                           /      \\
                          0     A3
                        (y=0)   /  \\
                            0 /    \\ 1
                             /      \\
                            0        1
                          (y=0)    (y=1)
"""

print(tree_ascii)
print()

# Alternative more detailed representation
tree_detailed = """
                           ROOT
                            |
                          [A2]
                         /    \\
                    A2=0/      \\A2=1
                      /        \\
                   [Class 0]    [A1]
                   (Pure)       /   \\
                          A1=0/     \\A1=1
                            /       \\
                      [Class 0]     [A3]
                      (Pure)        /  \\
                             A3=0 /    \\ A3=1
                               /        \\
                          [Class 0]  [Class 1]
                          (Pure)     (Pure)
"""

print("Detailed Tree Structure:")
print(tree_detailed)
print()

# Create a text representation
tree_text = """
1. If A2 == 0:
   └─ Predict Class 0

2. If A2 == 1:
   └─ If A1 == 0:
      └─ Predict Class 0
   └─ If A1 == 1:
      └─ If A3 == 0:
         └─ Predict Class 0
      └─ If A3 == 1:
         └─ Predict Class 1
"""

print("Tree Rules:")
print(tree_text)
print()

print("=" * 80)
print("SUMMARY OF REMAINING QUESTIONS")
print("=" * 80)
print()
print(f"Question 5 (First Split):  A2")
print(f"Question 6 (Second Split): {best_second}")
print()
