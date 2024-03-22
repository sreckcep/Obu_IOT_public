import time

#latest TAI information can be accessed at the IERS website
LATEST_LEAP_SECONDS_FILE_URL = "https://hpiers.obspm.fr/iers/bul/bulc/Leap_Second.dat"


def unix_to_tai_2004_milliseconds_time(unix_timestamp: float, leap_seconds: int) -> int:
    """Converts a unix timestamp to a timestampIts format

    Args:
        unix_timestamp (float): seconds since unix epoch
        leap_seconds (int): current amount of leap seconds between UTC and TAI

    Returns:
        int: amount of milliseconds since 2004-01-01 00:00:00 UTC
    """
    # TAI epoch starts on January 1, 2004
    tai_epoch = 1072915200  # Unix timestamp for 2004-01-01 00:00:00 UTC

    # Convert Unix timestamp to TAI timestamp
    tai_timestamp = unix_timestamp + (tai_epoch - leap_seconds)

    # Convert TAI timestamp to milliseconds
    tai_milliseconds = tai_timestamp * 1000

    return int(tai_milliseconds)


def main():
    
    unix_timestamp = time.time() 
    tai_milliseconds = int(unix_to_tai_2004_milliseconds_time(unix_timestamp, 37))
    tai_milliseconds_modulo = int(tai_milliseconds % 65536)
    print("UTC time: ", time.time())
    print("TAI since 2004 time value in milliseconds:", tai_milliseconds)
    print("CAM generationDeltaTime in milliseconds: ", tai_milliseconds_modulo)
    
if __name__ == "__main__":
    main()