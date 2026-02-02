"""
Utility functions: conversions and helpers.
"""

import pymap3d as pm3d
import numpy as np

def convert_search_area_polygon(poly_gps, reference):
    '''
    Purpose
    -------
    Convert GPS polygon coordinates to ENU.
    
    Inputs
    ------
    poly_gps : List[Tuple[float, float]]
        GPS coordinates of polygon.
    reference : Tuple[float, float, float]
        Reference GPS point.
    
    Outputs
    -------
    poly_corners_enu : List[Tuple[float, float]]
        ENU coordinates.
    poly_alt_ls : List[float]
        Altitudes.
    
    Notes
    -----
    - Handles rectangles (2 points) or polygons (3+ points).
    '''
    # Unpack reference point
    ref_lat, ref_lon, ref_alt = reference

    poly_corners_enu = []
    poly_alt_ls = []

    # ============================================================================
    # Rectangle from 2 diagonal points
    # ============================================================================
    if len(poly_gps) == 2:
        print("Rectangle as Input")
        # Extract the two diagonal corners
        (lat1, lon1), (lat2, lon2) = poly_gps


        # Convert both diagonal points to ENU
        e1, n1, u1 = pm3d.geodetic2enu(lat1, lon1, ref_alt, ref_lat, ref_lon, ref_alt)
        e2, n2, u2 = pm3d.geodetic2enu(lat2, lon2, ref_alt, ref_lat, ref_lon, ref_alt)

        poly_alt_ls = [u1, u1, u2, u2]


        # Calculate all four corners from the two diagonal points
        min_e = min(e1, e2)
        max_e = max(e1, e2)
        min_n = min(n1, n2)
        max_n = max(n1, n2)

        # Rectangle 
        poly_corners_enu = [ 
            (min_e, min_n),  # Bottom-left
            (max_e, min_n),  # Bottom-right
            (max_e, max_n),  # Top-right
            (min_e, max_n),  # Top-left
        ]

        return poly_corners_enu, poly_alt_ls
    
    # ============================================================================
    # Polygon from 3+ points
    # ============================================================================    
    elif len(poly_gps) >= 3:
        print("Polygon as Input")
        for lat, lon in poly_gps:
            e, n, u = pm3d.geodetic2enu(lat, lon, ref_alt, ref_lat, ref_lon, ref_alt)
            poly_corners_enu.append((e,n))
            poly_alt_ls.append(u)


        return poly_corners_enu, poly_alt_ls
    else:
        raise ValueError("polygon_gps must contain at least 2 points")

def convert_gps_to_rectangle(diagonal_gps, reference):
    """
    Convert two diagonal GPS points to ENU rectangle corners.
    
    Parameters
    ----------
    diagonal_gps : list of tuple
        Two GPS points (lat, lon) defining the rectangle diagonal.
    reference : tuple of float
        Reference GPS point (lat, lon, alt).
    
    Returns
    -------
    rectangle_corners : list of tuple
        Four ENU corners of the rectangle.
    u1 : float
        Altitude of first point.
    u2 : float
        Altitude of second point.
    
    Notes
    -----
    Calculates the four corners from the diagonal points.
    """
    # Unpack reference point
    ref_lat, ref_lon, ref_alt = reference

    # Extract the two diagonal corners
    (lat1, lon1), (lat2, lon2) = diagonal_gps
    
    
    # Convert both diagonal points to ENU
    e1, n1, u1 = pm3d.geodetic2enu(lat1, lon1, ref_alt, ref_lat, ref_lon, ref_alt)
    e2, n2, u2 = pm3d.geodetic2enu(lat2, lon2, ref_alt, ref_lat, ref_lon, ref_alt)
    
    # Calculate all four corners from the two diagonal points
    # Corner 1: (min_e, min_n) - bottom-left
    # Corner 2: (max_e, min_n) - bottom-right
    # Corner 3: (max_e, max_n) - top-right
    # Corner 4: (min_e, max_n) - top-left
    min_e = min(e1, e2)
    max_e = max(e1, e2)
    min_n = min(n1, n2)
    max_n = max(n1, n2)
    
    rectangle_corners = [
        (min_e, min_n),  # Bottom-left
        (max_e, min_n),  # Bottom-right
        (max_e, max_n),  # Top-right
        (min_e, max_n),  # Top-left
    ]
    
    return rectangle_corners, u1, u2

def convert_enu_to_gps(enu_point, reference):
    """
    Convert ENU coordinates to GPS (latitude, longitude, altitude).
    
    Parameters
    ----------
    enu_point : tuple of float
        ENU coordinates (e, n, u) in meters.
    reference : tuple of float
        Reference GPS point (lat, lon, alt) in degrees and meters.
    
    Returns
    -------
    tuple of float
        GPS coordinates (lat, lon, alt).
    
    Notes
    -----
    Uses pymap3d for geodetic conversions.
    """
    e, n, u = enu_point
    ref_lat, ref_lon, ref_alt = reference
    lat, lon, alt = pm3d.enu2geodetic(e, n, u, ref_lat, ref_lon, ref_alt, ell=None, deg=True)
    return lat, lon, alt

def convert_gps_point_to_enu(gps_point, reference):
    """
    Convert a GPS point to ENU coordinates.
    
    Parameters
    ----------
    gps_point : tuple of float
        GPS coordinates (lat, lon) in degrees.
    reference : tuple of float
        Reference GPS point (lat, lon, alt).
    
    Returns
    -------
    tuple of float
        ENU coordinates (e, n, u).
    
    Notes
    -----
    Uses pymap3d for geodetic conversions.
    """
    lat, lon = gps_point
    ref_lat, ref_lon, ref_alt = reference
    e, n, u = pm3d.geodetic2enu(lat, lon, ref_alt, ref_lat, ref_lon, ref_alt)
    return e, n, u