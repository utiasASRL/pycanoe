"""
Different sequence splits for odometry, localization, and object detection benchmarks.
Each split is a list of lists, where each inner list contains the sequence ID and optionally start and end timestamps (in microseconds) for the evaluation interval.
The following splits are provided for convenience:
- odom_sample: 1 sequence for quick testing of the CANOE dataset
- odom_test: 7 sequences for testing odometry methods
- loc_sample: 1 sequence for quick testing of localization methods
- loc_reference:   
    - 3 sequences serving as the reference for localization methods (i.e. the "map" sequence the other sequences are localized against)
    - reference sequences correspond to the matching route sequences in the loc_test split (e.g. canoe-2025-08-20-16-07 is the reference for canoe-2025-08-21-11-31)
- loc_test: 4 sequences for testing localization methods (does not include the loc_reference sequence)
"""
# CANOE dataset splits for odometry and localization benchmarks
# List: [Sequence ID]
odom_sample = [["canoe-2025-08-21-19-16"]]

# to do: confirm with the team that these are the final splits for the benchmark, and update if needed

odom_test = [
    ["canoe-2025-08-20-16-07"], # day 1 short
    ["canoe-2025-08-20-18-07"], # day 1 long
    ["canoe-2025-08-21-11-31"], # day 2 short
    ["canoe-2025-08-21-14-08"], # day 2 long
    ["canoe-2025-08-28-14-13"], # reservoir doppler 1
    ["canoe-2025-08-28-14-50"], # reservoir doppler 2
    ["canoe-2025-08-28-16-15"], # reservoir 3
]

loc_sample = odom_sample
loc_reference = [
    ["canoe-2025-08-20-16-07"], # day 1 short
    ["canoe-2025-08-20-18-07"], # day 1 long
    ["canoe-2025-08-28-14-50"], # reservoir doppler 2 
]

loc_test = [
    ["canoe-2025-08-21-11-31"], # day 2 short
    ["canoe-2025-08-21-14-08"], # day 2 long
    ["canoe-2025-08-28-14-13"], # reservoir doppler 1
    ["canoe-2025-08-28-16-15"], # reservoir 3
]