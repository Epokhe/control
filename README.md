# control

Solutions to the challenges on https://janismac.github.io/ControlChallenges/

https://github.com/user-attachments/assets/12383e1f-a750-41e8-a27e-32167a4c8a4a

The site was recently updated, but I wanted to continue from its old state, so ran the old version of the website locally.

Used the `027b7d091509f2fdbb9a73ad5b34b111128de7ab` hash from https://github.com/janismac/ControlChallenges

Just opening the index.html doesn't work, I was getting CORS errors and not seeing images. I don't remember the reasoning behind it but it was related to local websites accessed with `file://`. Easiest solution is running an http server on the root folder that serves the images. 

```
python -m http.server
```

I nearly had the solution for ball on platform 3, but I lost it long ago(removed from browser cache). If I start again, I want to do that, also the drone. Drone seems very similar to inverted pendulum, it should be easy.


