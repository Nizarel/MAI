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

# ============= ENTROPY CALCULATION =============
def calculate_entropy(y):
    """Calculate entropy of labels"""
    if len(y) == 0:
        return 0
    unique, counts = np.unique(y, return_counts=True)
    probabilities = counts / len(y)
    entropy = -np.sum([p * np.log2(p) for p in probabilities if p > 0])
    return entropy

# Calculate initial entropy
initial_entropy = calculate_entropy(y)

# ============= DETAILED CALCULATION FOR EACH ATTRIBUTE =============

print("=" * 80)
print("INFORMATION GAIN DETAILED CALCULATIONS")
print("=" * 80)
print()

# Initial State
print("Initial Dataset State:")
unique, counts = np.unique(y, return_counts=True)
print(f"  Total examples: {len(y)}")
print(f"  Class 0: {counts[0]} examples")
print(f"  Class 1: {counts[1]} examples")
print(f"  Initial Entropy H(S) = {initial_entropy:.4f}")
print()

# ============= QUESTION 2: Information Gain with A1 =============
print("=" * 80)
print("QUESTION 2: If we use A1 as first split feature")
print("=" * 80)

attr_idx = 0
attr_name = 'A1'
print(f"\nSplitting on attribute {attr_name}:")
print()

# Get unique values and split
attribute_values = X[:, attr_idx]
unique_vals = np.unique(attribute_values)

splits_a1 = {}
weighted_entropy_a1 = 0

for value in sorted(unique_vals):
    mask = attribute_values == value
    subset_indices = np.where(mask)[0]
    subset_y = y[mask]
    splits_a1[value] = subset_y
    
    # Count classes
    count_0 = np.sum(subset_y == 0)
    count_1 = np.sum(subset_y == 1)
    entropy = calculate_entropy(subset_y)
    
    print(f"{attr_name} = {int(value)}:")
    print(f"  Examples: {list(subset_indices + 1)} (x{subset_indices[0] + 1} to x{subset_indices[-1] + 1})")
    print(f"  Count: {len(subset_y)} examples")
    print(f"  Distribution: Class 0: {count_0}, Class 1: {count_1}")
    print(f"  Entropy H(S_{int(value)}) = {entropy:.4f}")
    print()
    
    weight = len(subset_y) / len(y)
    weighted_entropy_a1 += weight * entropy

ig_a1 = initial_entropy - weighted_entropy_a1
print(f"Weighted Entropy = Σ(|S_v|/|S| × H(S_v))")
print(f"                = {len(splits_a1[0])}/5 × {calculate_entropy(splits_a1[0]):.4f} + {len(splits_a1[1])}/5 × {calculate_entropy(splits_a1[1]):.4f}")
print(f"                = {len(splits_a1[0])/5:.1f} × {calculate_entropy(splits_a1[0]):.4f} + {len(splits_a1[1])/5:.1f} × {calculate_entropy(splits_a1[1]):.4f}")
print(f"                = {weighted_entropy_a1:.4f}")
print()
print(f"Information Gain IG(A1) = H(S) - Weighted Entropy")
print(f"                        = {initial_entropy:.4f} - {weighted_entropy_a1:.4f}")
print(f"                        = {ig_a1:.4f}")
print()
print(f"ANSWER TO QUESTION 2: {ig_a1:.4f}")
print()

# ============= QUESTION 3: Information Gain with A2 =============
print("=" * 80)
print("QUESTION 3: If we use A2 as first split feature")
print("=" * 80)

attr_idx = 1
attr_name = 'A2'
print(f"\nSplitting on attribute {attr_name}:")
print()

# Get unique values and split
attribute_values = X[:, attr_idx]
unique_vals = np.unique(attribute_values)

splits_a2 = {}
weighted_entropy_a2 = 0

for value in sorted(unique_vals):
    mask = attribute_values == value
    subset_indices = np.where(mask)[0]
    subset_y = y[mask]
    splits_a2[value] = subset_y
    
    # Count classes
    count_0 = np.sum(subset_y == 0)
    count_1 = np.sum(subset_y == 1)
    entropy = calculate_entropy(subset_y)
    
    print(f"{attr_name} = {int(value)}:")
    print(f"  Examples: {list(subset_indices + 1)} (x{subset_indices[0] + 1} to x{subset_indices[-1] + 1})")
    print(f"  Count: {len(subset_y)} examples")
    print(f"  Distribution: Class 0: {count_0}, Class 1: {count_1}")
    print(f"  Entropy H(S_{int(value)}) = {entropy:.4f}")
    print()
    
    weight = len(subset_y) / len(y)
    weighted_entropy_a2 += weight * entropy

ig_a2 = initial_entropy - weighted_entropy_a2
print(f"Weighted Entropy = Σ(|S_v|/|S| × H(S_v))")
print(f"                = {len(splits_a2[0])}/5 × {calculate_entropy(splits_a2[0]):.4f} + {len(splits_a2[1])}/5 × {calculate_entropy(splits_a2[1]):.4f}")
print(f"                = {len(splits_a2[0])/5:.1f} × {calculate_entropy(splits_a2[0]):.4f} + {len(splits_a2[1])/5:.1f} × {calculate_entropy(splits_a2[1]):.4f}")
print(f"                = {weighted_entropy_a2:.4f}")
print()
print(f"Information Gain IG(A2) = H(S) - Weighted Entropy")
print(f"                        = {initial_entropy:.4f} - {weighted_entropy_a2:.4f}")
print(f"                        = {ig_a2:.4f}")
print()
print(f"ANSWER TO QUESTION 3: {ig_a2:.4f}")
print()

# ============= QUESTION 4: Information Gain with A3 =============
print("=" * 80)
print("QUESTION 4: If we use A3 as first split feature")
print("=" * 80)

attr_idx = 2
attr_name = 'A3'
print(f"\nSplitting on attribute {attr_name}:")
print()

# Get unique values and split
attribute_values = X[:, attr_idx]
unique_vals = np.unique(attribute_values)

splits_a3 = {}
weighted_entropy_a3 = 0

for value in sorted(unique_vals):
    mask = attribute_values == value
    subset_indices = np.where(mask)[0]
    subset_y = y[mask]
    splits_a3[value] = subset_y
    
    # Count classes
    count_0 = np.sum(subset_y == 0)
    count_1 = np.sum(subset_y == 1)
    entropy = calculate_entropy(subset_y)
    
    print(f"{attr_name} = {int(value)}:")
    print(f"  Examples: {list(subset_indices + 1)} (x{subset_indices[0] + 1} to x{subset_indices[-1] + 1})")
    print(f"  Count: {len(subset_y)} examples")
    print(f"  Distribution: Class 0: {count_0}, Class 1: {count_1}")
    print(f"  Entropy H(S_{int(value)}) = {entropy:.4f}")
    print()
    
    weight = len(subset_y) / len(y)
    weighted_entropy_a3 += weight * entropy

ig_a3 = initial_entropy - weighted_entropy_a3
print(f"Weighted Entropy = Σ(|S_v|/|S| × H(S_v))")
print(f"                = {len(splits_a3[0])}/5 × {calculate_entropy(splits_a3[0]):.4f} + {len(splits_a3[1])}/5 × {calculate_entropy(splits_a3[1]):.4f}")
print(f"                = {len(splits_a3[0])/5:.1f} × {calculate_entropy(splits_a3[0]):.4f} + {len(splits_a3[1])/5:.1f} × {calculate_entropy(splits_a3[1]):.4f}")
print(f"                = {weighted_entropy_a3:.4f}")
print()
print(f"Information Gain IG(A3) = H(S) - Weighted Entropy")
print(f"                        = {initial_entropy:.4f} - {weighted_entropy_a3:.4f}")
print(f"                        = {ig_a3:.4f}")
print()
print(f"ANSWER TO QUESTION 4: {ig_a3:.4f}")
print()

# ============= SUMMARY =============
print("=" * 80)
print("SUMMARY OF ALL QUESTIONS")
print("=" * 80)
print()
print(f"Question 1 (Initial Entropy):  H(S) = {initial_entropy:.4f}")
print()
print(f"Question 2 (IG with A1):       IG(A1) = {ig_a1:.4f}")
print(f"Question 3 (IG with A2):       IG(A2) = {ig_a2:.4f}")
print(f"Question 4 (IG with A3):       IG(A3) = {ig_a3:.4f}")
print()
print(f"Best Split: A2 with IG = {max(ig_a1, ig_a2, ig_a3):.4f}")
print()
