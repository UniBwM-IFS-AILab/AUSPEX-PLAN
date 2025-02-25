import numpy as np
from scipy.stats import rv_discrete

def pmf(distribution, value):
    """
    Compute the probability density for `value` in a continuous distribution.
    
    :param distribution: A `scipy.stats` distribution object (like norm, uniform, etc.).
    :param value: The value for which the probability density is requested.
    :return: The probability density of the value.
    """
    return distribution.pmf(value)


def get_an(G, index):
    return G.action_nodes[index]  #

def get_bn(G, index):
    return G.belief_nodes[index]  #


def weighted_iterator_old(distribution, size=100):
    # Ensure that we are working with a distribution object
    if not hasattr(distribution, 'rvs') or not hasattr(distribution, 'pmf'):
        print(type(distribution))
        raise ValueError("The provided object is not a valid distribution object.")
    
    # Generate samples
    samples = distribution.rvs(size=size)
    for sample in samples:
        yield sample, distribution.pmf(sample)


def weighted_iterator(d):
    """
    Create an iterator for a distribution where each element is associated with its probability density function value.
    
    Parameters:
    - d: Distribution object (or any object with an 'iterator' method and a 'pdf' method)
    
    Returns:
    - A generator yielding tuples of (element, pdf_value)
    """
    #iterator = d.support()
    values = range(d.a, d.b + 1)
    # Use a generator to yield the value and its corresponding PMF
    return ((x, pmf(d, x)) for x in values)

# def weighted_iterator(d):
#     """
#     Create an iterator for a distribution where each element is associated with its probability density function value.
    
#     Parameters:
#     - d: Distribution object (or any object with an 'iterator' method and a 'pdf' method)
    
#     Returns:
#     - A generator yielding tuples of (element, pdf_value)
#     """
#     iterator = d.support()  # Assuming 'support()' gives an iterable of values
#     # Convert support to a numpy array
#     iterator = np.array(iterator)
    
#     # Vectorized PMF computation
#     pmf_values = d.pmf(iterator)
    
#     # Return a generator yielding (element, pmf_value) pairs
#     return zip(iterator, pmf_values)