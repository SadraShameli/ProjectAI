namespace ProjectAI
{
    template <typename T>
    inline static void RemoveWhiteSpace(T &t)
    {
        t.erase(t.find_last_not_of(" \n\r\t") + 1);
    }

    inline static long MapValue(long x, long in_min, long in_max, long out_min, long out_max)
    {
        long temp = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        return std::clamp(temp, out_min, out_max);
    }
}
