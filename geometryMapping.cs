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
        private static ComplexNum FLTGamma(float theta, float time) {
            ComplexNum numerator = new ComplexNum(
                Mathf.Exp(time)*Mathf.Cos(theta/2),
                Mathf.Sin(theta/2)
            );
            ComplexNum denominator = new ComplexNum(
                Mathf.Exp(time)*Mathf.Sin(theta/2),
                -1 * Mathf.Cos(theta/2)
            );
            return numerator/denominator;
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
                    float phi = Mathf.Atan2(whatDirectionImShooting.z, whereImShootingFrom.x);
                    float theta = Mathf.Acos(whatDirectionImShooting.y);
                    ComplexNum geodesic = FLTGamma(
                        theta, t
                    );

                    // Mostly right, just need to fix rotation
                    Vector3 positionMapped = new Vector3(
                        Mathf.Cos(phi)*geodesic.r*whereImShootingFrom.y + whereImShootingFrom.x,
                        geodesic.i*whereImShootingFrom.y,
                        Mathf.Sin(phi)*geodesic.r*whereImShootingFrom.y + whereImShootingFrom.z
                    );

                    return positionMapped;

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