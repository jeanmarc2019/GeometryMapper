using System.Collections.Generic;
using System;
using UnityEngine;

namespace GeometryMapper
{
    class PhysicsHelper
    {
        public static string currentGeometry = "Half-Plane"; // default is Nil geometry

        public static void changeGeometry(String geometry)
        {
            currentGeometry = geometry;
        }

        private static Vector3 NilMultiply(Vector3 left, Vector3 right)
        {
            Vector3 normalSum = new Vector3(
                left.x + right.x,
                left.y + right.y + (left.x*right.z - right.x*left.z),
                left.z + right.z
            );
            return normalSum;
        }
        private static Vector3 RussianArrowFromOrigin(float r, float varphi, float gamma, float t)
        {
            float x = (r/(2*gamma))*(Mathf.Sin(2*gamma*t + varphi) - Mathf.Sin(varphi));
            float y = ((1+Mathf.Pow(gamma, 2f))/(2*gamma))*t - ((1 - Mathf.Pow(gamma, 2f))/(4*Mathf.Pow(gamma, 2f)))*Mathf.Sin(2*gamma*t);
            float z = (r/(2*gamma))*(Mathf.Cos(varphi) - Mathf.Cos(2*gamma*t + varphi));
            Vector3 output = new Vector3(x,y,z);
            return output;
        }
        public static float aFromDirection(Vector3 direction)
        {
            return (float)Math.Pow(Mathf.Pow(direction.x, 2f) + Mathf.Pow(direction.z, 2f), 0.5f);
        }
        public static float angleFromDirection(Vector3 direction)
        {
            return (float)Mathf.Atan2(direction.z, direction.x);
        }
        public static Vector3 ArrowFromOrigin(Vector3 input, float t)
        {
            return RussianArrowFromOrigin(aFromDirection(input), angleFromDirection(input), input.y, t);
        }
        private static Vector3 BringVelocityToOrigin(Vector3 velocity, Vector3 oldLocation)
        {
            return NilMultiply(-1*oldLocation, oldLocation + velocity);
        }
        private static Vector3 BeingVelocityToLocation(Vector3 velocity, Vector3 newLocation)
        {
            return -1*newLocation + NilMultiply(newLocation, velocity);
        }
        private static Vector3 getRotationAxis(Vector3 input) {
            Vector2 projected = new Vector2(input.x, input.z);
            Vector2 orthogonal = Vector2.Perpendicular(projected);
            return new Vector3(orthogonal.x, 0, orthogonal.y);
        }
        public static Vector3 positionMap(Vector3 whereImShootingFrom, Vector3 whatDirectionImShooting, float t)
        {

            switch(currentGeometry) {
                case "Nil":
                    return NilMultiply(
                        whereImShootingFrom,
                        ArrowFromOrigin(
                            BringVelocityToOrigin(whatDirectionImShooting, whereImShootingFrom),
                            t
                        )
                    );
                case "Half-Plane":

                    // test logs
                    float testDir1 = (float)Mathf.Pow(2, .5f)/2;
                    float testDir2 = (float)Mathf.Pow(2, .5f)/2;
                    Vector3 test = new Vector3(testDir1, testDir2, 0);
                    Debug.Log(test);
                    Vector3 test2 = Quaternion.AngleAxis(90, getRotationAxis(test)) * test;
                    Vector3 testProjection = new Vector3(test2.x, 0, test2.z);
                    float testAngleBetweenXZandUp = Mathf.Acos(
                           (Vector3.Dot(test2, testProjection))/(test2.magnitude * testProjection.magnitude)
                       );
                    Vector3 testFlatDir = new Vector3(test.x, 0, test.z);
                    float testRadius = test.y / Mathf.Cos(testAngleBetweenXZandUp);

                    float testAngleBetweenXZandUpXandDir = Mathf.Acos(
                        ((Vector3.Dot(testFlatDir, new Vector3(1,0,0)))/(testFlatDir.magnitude))
                    );
                    Vector3 testMapped = new Vector3(
                      Mathf.Cos(testAngleBetweenXZandUpXandDir) * Mathf.Cos(testAngleBetweenXZandUp),
                      Mathf.Sin(testAngleBetweenXZandUp),
                      Mathf.Sin(testAngleBetweenXZandUpXandDir) * Mathf.Cos(testAngleBetweenXZandUp)
                    );
                    Debug.Log(testMapped);


                    // supposed to rotate the vector up 90 degrees, finding the rotation axis using the perpendicular line in the plane
                    Vector3 up =  Quaternion.AngleAxis(90, getRotationAxis(whatDirectionImShooting)) * whatDirectionImShooting;
                    Vector3 projection = new Vector3(up.x, 0, up.z);
                    float angleBetweenXZandUp = Mathf.Acos(
                        (Vector3.Dot(up, projection))/(up.magnitude * projection.magnitude)
                    );

                    // projects the direction shot to the xz-axis (the plane)
                    Vector3 flatDir = new Vector3(whatDirectionImShooting.x, 0, whatDirectionImShooting.z);

                    float radius = whatDirectionImShooting.y / Mathf.Cos(angleBetweenXZandUp);

                    float angleBetweenXandDir = Mathf.Acos(
                        ((Vector3.Dot(flatDir, new Vector3(1,0,0)))/(flatDir.magnitude))
                    );



//                    Vector3 projection2 = new Vector3(whatDirectionImShooting.x, 0, whatDirectionImShooting.z);
//                    float angleBetweenXZandDir = Mathf.Acos(
//                        (Vector3.Dot(whatDirectionImShooting, projection2))/(whatDirectionImShooting.magnitude * projection2.magnitude)
//                    );



                    Vector3 positionMapped = new Vector3(
                      Mathf.Cos(angleBetweenXandDir) * Mathf.Cos(angleBetweenXZandUp - t),
                      Mathf.Sin(angleBetweenXZandUp - t),
                      Mathf.Sin(angleBetweenXandDir) * Mathf.Cos(angleBetweenXZandUp - t)
                    );
                    return whereImShootingFrom + positionMapped * radius;

                default:
                    return whereImShootingFrom + whatDirectionImShooting*t;
            }
        }
        public static Vector3 derivateApprox(Vector3 position, Vector3 direction, float t)
        {
            Vector3 f_of_x_plus_h = positionMap(position, direction, t + Time.deltaTime);
            Vector3 f_of_x = positionMap(position, direction, t);
            float h = Time.deltaTime;
            return (f_of_x_plus_h - f_of_x)/h;
        }
    }
}