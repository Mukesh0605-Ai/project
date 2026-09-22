# EKF Design Document

**STATUS:** PENDING DATASET AILGNMENT

## State Vector
- **UNKNOWN**. Pending inspection of `IO-VNBD` dataset.

## Coordinate Frame
- **UNKNOWN**.

## Orientation Representation
- **UNKNOWN**.

## Process Model
- **PENDING**. Will propagate orientation, velocity, and position based on IMU integration. Exact equations blocked by unknown frame and unit alignment.

## Measurement Model
- **PENDING**. GNSS update equations will be formulated once the GNSS field format is verified.

## Covariance Representation
- **PENDING**.

## GNSS Outage and Recovery
- The filter is structurally designed to run purely on the Process Model during `DENIED` states and apply gated measurement updates during `RECOVERY` states. Implementation is deferred.
