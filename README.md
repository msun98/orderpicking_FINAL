# orderpicking_FINAL
```
// calc dists and heading angle
    for(size_t p = 0; p < res.size()-1; p++)
    {
        double dx = res[p+1].pt[0]-res[p].pt[0];
        double dy = res[p+1].pt[1]-res[p].pt[1];
        double d = std::sqrt(dx*dx + dy*dy);
        double th = std::atan2(dy, dx);

        res[p+1].od = res[p].od + d;
        res[p].th = th;
        res[p+1].th = th;
    }
```
