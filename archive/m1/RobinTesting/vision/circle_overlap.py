import numpy as np

def random_sampling_overlap(mask, center, radius, num_samples=100):
    """
    Computes the percentage overlap of a circle with a binary mask using random sampling.
    
    Parameters:
    - mask: Binary mask (numpy array) where non-zero values represent areas of interest.
    - center: Tuple (x, y) representing the center of the circle.
    - radius: Radius of the circle.
    - num_samples: Number of random samples to take within the circle.
    
    Returns:
    - percentage_overlap: The fraction of sampled points within the mask.
    """
    # Generate random angles and radii for sampling points within the circle
    angles = np.random.uniform(0, 2 * np.pi, num_samples)  # Random angles from 0 to 2π
    radii = np.random.uniform(0, radius, num_samples)  # Random radii from 0 to the circle's radius
    
    # Convert polar coordinates (angles, radii) to Cartesian coordinates (x, y)
    sample_x = center[0] + (radii * np.cos(angles))
    sample_y = center[1] + (radii * np.sin(angles))
    
    # Ensure sampled points are within image bounds by clipping
    valid_x = np.clip(sample_x, 0, mask.shape[1] - 1).astype(int)
    valid_y = np.clip(sample_y, 0, mask.shape[0] - 1).astype(int)
    
    # Count the number of sampled points that overlap with the mask
    overlap_count = np.count_nonzero(mask[valid_y, valid_x] > 0)
    
    # Calculate the degree of overlap as a percentage of total samples
    percentage_overlap = overlap_count / num_samples
    
    return percentage_overlap
