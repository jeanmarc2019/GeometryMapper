using System.Collections.Generic;
using System;
using UnityEngine;

namespace GeometryMapper
{
    class PhysicsHelper
    {
        public static string currentGeometry = "Euclidean"; // default is Euclidean geometry

        public static void changeGeometry(String geometry)
        {
            Debug.Log("Geometry set to " + geometry);
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
        private static Vector3 nilGeodesicEqn(float r, float varphi, float gamma, float t)
        {
            float x = (r/(2*gamma))*(Mathf.Sin(2*gamma*t + varphi) - Mathf.Sin(varphi));
            float y = ((1+Mathf.Pow(gamma, 2f))/(2*gamma))*t - ((1 - Mathf.Pow(gamma, 2f))/(4*Mathf.Pow(gamma, 2f)))*Mathf.Sin(2*gamma*t);
            float z = (r/(2*gamma))*(Mathf.Cos(varphi) - Mathf.Cos(2*gamma*t + varphi));
            Vector3 output = new Vector3(x,y,z);
            return output;
        }
        private static Vector3 getRotationAxis(Vector3 input) {
            Vector2 projected = new Vector2(input.x, input.z);
            Vector2 orthogonal = Vector2.Perpendicular(projected);
            return new Vector3(orthogonal.x, 0, orthogonal.y);
        }
        private static ComplexNum FLTGamma(float theta, float time)
        {
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
                    Vector3 velocityAtOrigin = NilMultiply(-1*whereImShootingFrom, whereImShootingFrom + whatDirectionImShooting);
                    float r = (float)Math.Pow(Mathf.Pow(velocityAtOrigin.x, 2f) + Mathf.Pow(velocityAtOrigin.z, 2f), 0.5f);
                    float varphi = (float)Mathf.Atan2(velocityAtOrigin.z, velocityAtOrigin.x);
                    float gamma = velocityAtOrigin.y;
                    return NilMultiply(
                        whereImShootingFrom,
                        nilGeodesicEqn(r, varphi, gamma, t)
                    );
                case "Half-Space":
                    float phi = Mathf.Atan2(whatDirectionImShooting.z, whatDirectionImShooting.x);
                    float theta = Mathf.Acos(whatDirectionImShooting.y);
                    ComplexNum geodesic = FLTGamma(
                        theta, t
                    );
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