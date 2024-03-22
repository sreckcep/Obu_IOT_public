from dataclasses import dataclass, field
from enum import Enum

class TrafficParticipantType(int, Enum):
    unknown         : int = 0
    pedestrian      : int = 1
    cyclist         : int = 2
    moped           : int = 3
    motorcycle      : int = 4
    passengerCar    : int = 5
    bus             : int = 6
    lightTruck      : int = 7
    heavyTruck      : int = 8
    trailer         : int = 9
    specialVehicle  : int = 10
    tram            : int = 11
    lightVruVehicle : int = 12
    animal          : int = 13
    agricultural    : int = 14
    roadSideUnit    : int = 15

class MessageID(int, Enum):
    denm : int              = 1 
    cam : int               = 2
    poi : int               = 3
    spatem : int            = 4
    mapem : int             = 5
    ivim : int              = 6
    ev_rsr : int            = 7
    tistpgtransaction : int = 8
    srem : int              = 9
    ssem : int              = 10 
    evcsn : int             = 11 
    saem : int              = 12 
    rtcmem : int            = 13 
    cpm : int               = 14
    imzm : int              = 15
    vam : int               = 16
    dsm : int               = 17 
    pcim : int              = 18
    pcvm : int              = 19
    mcm : int               = 20
    pam : int               = 21
@dataclass

class ReferencePositionWithConfidence:
    lat: int = 900_000_001
    lon: int = 900_000_001
    alt: int = 0
    hAcc: int = 0
    
@dataclass
class BasicContainer:
    trafficParticipantType: TrafficParticipantType = TrafficParticipantType.unknown
    referencePosition: ReferencePositionWithConfidence = field(default_factory=ReferencePositionWithConfidence)

@dataclass
class Heading:
    headingValue: int = 3601
    headingConfidence: int = 127
@dataclass   
class Speed:
    speedValue: int = 16383
    speedConfidence: int = 127
@dataclass
class Curvature:
    curvatureValue: int = 1023
    curvatureConfidence: int = 7
@dataclass
class LongitudinalAcceleration:
    longitudinalAccelerationValue: int = 161
    longitudinalAccelerationConfidence: int = 102
@dataclass
class YawRate:
    yawRateValue: int = 32767
    yawRateConfidence: int = 8  
@dataclass
class VehicleLength:
    vehicleLengthValue : int = 1023
    vehicleLengthConfidenceIndication: int = 4
    
@dataclass
class HighFrequencyContainer:
    heading : Heading = field(default_factory=Heading)
    speed : Speed = field(default_factory=Speed)
    driveDirection: int = 2
    vehicleLength: VehicleLength = field(default_factory=VehicleLength)
    vehicleWidth: int = 62
    longitudinalAcceleration: LongitudinalAcceleration = field(default_factory=LongitudinalAcceleration)
    curvature: Curvature = field(default_factory=Curvature)
    curvatureCalculationMode: int = 2
    yawRate: YawRate = field(default_factory=YawRate)
    
@dataclass
class CamParameters:
    basicContainer: BasicContainer = field(default_factory=BasicContainer)
    highFrequencyContainer: HighFrequencyContainer = field(default_factory=HighFrequencyContainer)
    
@dataclass
class ItsPduHeader:
    protocolVersion: int = 0
    messageId: int = 0
    stationId: int = 0

@dataclass
class CamPayload:
    generationDeltaTime: int = 0
    camParameters: CamParameters = field(default_factory=CamParameters)

@dataclass
class CAM:
    itsPduHeader: ItsPduHeader = field(default_factory=ItsPduHeader)
    camPayload: CamPayload = field(default_factory=CamPayload)