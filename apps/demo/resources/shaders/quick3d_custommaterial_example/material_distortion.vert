void MAIN()
{
    VERTEX.y += sin(uTime + VERTEX.x*10.0) * cos(uTime + VERTEX.z*10.0)* uAmplitude;
}
