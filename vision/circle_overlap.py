import numpy as np

def random_sampling_overlap(mask, center, radius, num_samples=100):
    # Generate random angles and radii for the samples
    angles = np.random.uniform(0, 2 * np.pi, num_samples)
    radii = np.random.uniform(0, radius, num_samples) 
    
    # Convert polar coordinates to Cartesian coordinates
    sample_x = center[0] + (radii * np.cos(angles)).astype(int)
    sample_y = center[1] + (radii * np.sin(angles)).astype(int)
    
    # Ensure the sampled points are within the image bounds
    valid_x = np.clip(sample_x, 0, mask.shape[1] - 1)
    valid_y = np.clip(sample_y, 0, mask.shape[0] - 1)
    
    # Check if the sampled points overlap with the mask
    overlap_count = np.count_nonzero(mask[valid_y, valid_x] > 0)
    
    # Calculate the degree of overlap
    percentage_overlap = overlap_count / num_samples
    
    return percentage_overlap