package main

import (
	"encoding/json"
	"fmt"
	"math"
	"math/rand"
	"os"
	"runtime"
	"sync"
)

const (
	G                = 9.81
	SimStep          = 0.01
	MaxTime          = 600
	MaxAccel         = 1 * G
	ShipRadius       = 500.0
	CubeSize         = 1e5 // 100 000 км в метрах
	Pellets          = 20
	SpreadAngle      = math.Pi / 4 // 45°
	EffectiveDist    = 2e7         // 20 000 км
	NWorkers         = 8           // Количество параллельных горутин
	PelletsBatchSize = 5000
	ProjectileV      = 3500
)

type ShipSnapshot struct {
	Name  string  `json:"name"`
	X     float64 `json:"x"`
	Y     float64 `json:"y"`
	Z     float64 `json:"z"`
	HP    float64 `json:"hp"`
	Alive bool    `json:"alive"`
}
type ProjectileSnapshot struct {
	X     float64 `json:"x"`
	Y     float64 `json:"y"`
	Z     float64 `json:"z"`
	Owner string  `json:"owner"`
}
type Frame struct {
	T           float64              `json:"t"`
	Ships       []ShipSnapshot       `json:"ships"`
	Projectiles []ProjectileSnapshot `json:"projectiles"`
}

type Vec3 struct {
	X, Y, Z float64
}

func (v Vec3) Add(b Vec3) Vec3      { return Vec3{v.X + b.X, v.Y + b.Y, v.Z + b.Z} }
func (v Vec3) Sub(b Vec3) Vec3      { return Vec3{v.X - b.X, v.Y - b.Y, v.Z - b.Z} }
func (v Vec3) Len() float64         { return math.Sqrt(v.X*v.X + v.Y*v.Y + v.Z*v.Z) }
func (v Vec3) Scale(f float64) Vec3 { return Vec3{v.X * f, v.Y * f, v.Z * f} }
func (v Vec3) Norm() Vec3 {
	l := v.Len()
	if l == 0 {
		return Vec3{0, 0, 0}
	}
	return v.Scale(1.0 / l)
}

// Вращение в 3D вокруг случайной оси на угол (для рассевания)
func (v Vec3) Rotated(spread float64) Vec3 {
	theta := rand.NormFloat64() * (spread / 2)
	phi := rand.Float64() * 2 * math.Pi
	return Vec3{
		math.Sin(theta) * math.Cos(phi),
		math.Sin(theta) * math.Sin(phi),
		math.Cos(theta),
	}.Norm()
}

type Gun struct {
	ROF         float64
	ProjectileV float64
	Damage      float64
	Reload      float64
}

type Projectile struct {
	Pos        Vec3
	Vel        Vec3
	Target     *Ship
	Owner      *Ship
	Damage     float64
	Alive      bool
	FlightTime float64
}

type Ship struct {
	Name    string
	Pos     Vec3
	Vel     Vec3
	Armor   float64
	Guns    []Gun
	Alive   bool
	Target  *Ship
	Evasion float64
}

type Hit struct {
	Target *Ship
	Damage float64
}

func main() {
	ship1 := Ship{
		Name:    "Orion",
		Pos:     Vec3{0, 0, 0},
		Vel:     Vec3{0, 0, 0},
		Armor:   300,
		Guns:    []Gun{{ROF: 1, ProjectileV: ProjectileV, Damage: 240}},
		Alive:   true,
		Evasion: MaxAccel,
	}
	ship2 := Ship{
		Name:    "Valkyrie",
		Pos:     Vec3{CubeSize, CubeSize, CubeSize},
		Vel:     Vec3{0, 0, 0},
		Armor:   300,
		Guns:    []Gun{{ROF: 1, ProjectileV: ProjectileV, Damage: 240}},
		Alive:   true,
		Evasion: MaxAccel,
	}
	ship1.Target = &ship2
	ship2.Target = &ship1

	projectiles := []Projectile{}
	t := 0.0

	var frames []Frame
	for ; t < MaxTime && ship1.Alive && ship2.Alive; t += SimStep {
		SmartManeuver3D(&ship1, projectiles)
		SmartManeuver3D(&ship2, projectiles)
		UpdatePhysics3D(&ship1)
		UpdatePhysics3D(&ship2)
		FireShotgun3D(&ship1, &ship2, &projectiles, Pellets, SpreadAngle)
		FireShotgun3D(&ship2, &ship1, &projectiles, Pellets, SpreadAngle)
		projectiles, hits := UpdateProjectiles3DWorkerPool(projectiles, PelletsBatchSize)
		ApplyHits(hits)
		if int(t*10)%100 == 0 {
			fmt.Printf("t=%.0fs: %s(%.1f) <-> %s(%.1f), dist=%.1fкм, активных пуль: %d\n",
				t, ship1.Name, ship1.Armor, ship2.Name, ship2.Armor, ship1.Pos.Sub(ship2.Pos).Len()/1000, len(projectiles))
		}
		frame := Frame{
			T: t,
			Ships: []ShipSnapshot{
				{
					Name:  ship1.Name,
					X:     ship1.Pos.X,
					Y:     ship1.Pos.Y,
					Z:     ship1.Pos.Z,
					HP:    ship1.Armor,
					Alive: ship1.Alive,
				},
				{
					Name:  ship2.Name,
					X:     ship2.Pos.X,
					Y:     ship2.Pos.Y,
					Z:     ship2.Pos.Z,
					HP:    ship2.Armor,
					Alive: ship2.Alive,
				},
			},
			Projectiles: make([]ProjectileSnapshot, 0, len(projectiles)),
		}
		if int(t*10)%10 == 0 { // или просто if tick % N == 0
			for _, p := range projectiles {
				if p.Alive {
					frame.Projectiles = append(frame.Projectiles, ProjectileSnapshot{X: p.Pos.X, Y: p.Pos.Y, Z: p.Pos.Z, Owner: p.Owner.Name})
				}
			}

			frames = append(frames, frame)
		}
	}
	fmt.Printf("\nФИНАЛ:\n")
	fmt.Printf("%s: броня %.1f, живой: %v\n", ship1.Name, ship1.Armor, ship1.Alive)
	fmt.Printf("%s: броня %.1f, живой: %v\n", ship2.Name, ship2.Armor, ship2.Alive)

	f, err := os.Create("battle.json")
	if err != nil {
		panic(err)
	}
	defer f.Close()
	enc := json.NewEncoder(f)
	enc.SetIndent("", "  ")
	if err := enc.Encode(frames); err != nil {
		panic(err)
	}

}

// Манёвры: либо уклоняемся, либо летим к цели
func SmartManeuver3D(ship *Ship, projectiles []Projectile) {
	nearest := -1.0
	evadeVec := Vec3{}
	for _, p := range projectiles {
		if !p.Alive || p.Target != ship {
			continue
		}
		relPos := p.Pos.Sub(ship.Pos)
		relVel := p.Vel.Sub(ship.Vel)
		dot := relPos.X*relVel.X + relPos.Y*relVel.Y + relPos.Z*relVel.Z
		if dot >= 0 {
			continue
		}
		tImpact := -dot / (relVel.X*relVel.X + relVel.Y*relVel.Y + relVel.Z*relVel.Z)
		if tImpact < 0 || tImpact > 10 {
			continue
		}
		maxOffset := 0.5 * MaxAccel * tImpact * tImpact
		cross := Vec3{
			relPos.Y*relVel.Z - relPos.Z*relVel.Y,
			relPos.Z*relVel.X - relPos.X*relVel.Z,
			relPos.X*relVel.Y - relPos.Y*relVel.X,
		}
		distToTraj := cross.Len() / relVel.Len()
		if maxOffset > distToTraj-ShipRadius {
			randVec := Vec3{rand.Float64() - 0.5, rand.Float64() - 0.5, rand.Float64() - 0.5}.Norm()
			ortho := relVel.Norm()
			evade := Vec3{
				ortho.Y*randVec.Z - ortho.Z*randVec.Y,
				ortho.Z*randVec.X - ortho.X*randVec.Z,
				ortho.X*randVec.Y - ortho.Y*randVec.X,
			}.Norm().Scale(MaxAccel * SimStep)
			if nearest < 0 || tImpact < nearest {
				evadeVec = evade
				nearest = tImpact
			}
		}
	}
	if nearest < 0 {
		if ship.Target != nil && ship.Target.Alive {
			dir := ship.Target.Pos.Sub(ship.Pos).Norm()
			ship.Vel = ship.Vel.Add(dir.Scale(MaxAccel * SimStep))
		}
	} else {
		ship.Vel = ship.Vel.Add(evadeVec)
	}
}

func UpdatePhysics3D(ship *Ship) {
	ship.Pos = ship.Pos.Add(ship.Vel.Scale(SimStep))
}

// Веерная стрельба в 3D
func FireShotgun3D(attacker, defender *Ship, projectiles *[]Projectile, nPellets int, spreadAngle float64) {
	if !attacker.Alive || !defender.Alive {
		return
	}
	dist := attacker.Pos.Sub(defender.Pos).Len()
	if dist > EffectiveDist {
		return
	}
	for i := range attacker.Guns {
		gun := &attacker.Guns[i]
		gun.Reload -= SimStep
		if gun.Reload <= 0 {
			gun.Reload = 1.0 / gun.ROF
			dir := defender.Pos.Sub(attacker.Pos).Norm()
			for n := 0; n < nPellets; n++ {
				spreadDir := Random3DSpread(dir, spreadAngle)
				projVel := spreadDir.Scale(gun.ProjectileV).Add(attacker.Vel)
				*projectiles = append(*projectiles, Projectile{
					Pos:    attacker.Pos,
					Vel:    projVel,
					Target: defender,
					Damage: gun.Damage / float64(nPellets),
					Alive:  true,
					Owner:  attacker,
				})
			}
		}
	}
}

// 3D рассеивание вокруг основного направления
func Random3DSpread(dir Vec3, angle float64) Vec3 {
	var ortho1 Vec3
	if math.Abs(dir.X) < 0.5 {
		ortho1 = Vec3{1, 0, 0}
	} else if math.Abs(dir.Y) < 0.5 {
		ortho1 = Vec3{0, 1, 0}
	} else {
		ortho1 = Vec3{0, 0, 1}
	}
	ortho1 = Cross(dir, ortho1).Norm()
	ortho2 := Cross(dir, ortho1).Norm()
	phi := rand.Float64() * 2 * math.Pi
	theta := (rand.Float64() - 0.5) * angle
	return dir.Scale(math.Cos(theta)).
		Add(ortho1.Scale(math.Sin(theta) * math.Cos(phi))).
		Add(ortho2.Scale(math.Sin(theta) * math.Sin(phi))).Norm()
}

// Векторное произведение
func Cross(a, b Vec3) Vec3 {
	return Vec3{
		a.Y*b.Z - a.Z*b.Y,
		a.Z*b.X - a.X*b.Z,
		a.X*b.Y - a.Y*b.X,
	}
}

// --- ПАРАЛЛЕЛЬНЫЙ АПДЕЙТ СНАРЯДОВ ---
func UpdateProjectiles3DParallel(projectiles []Projectile) ([]Projectile, []Hit) {
	type result struct {
		newProj []Projectile
		hits    []Hit
	}
	var wg sync.WaitGroup
	nThreads := NWorkers
	chunkSize := (len(projectiles) + nThreads - 1) / nThreads
	resChan := make(chan result, nThreads)

	for t := 0; t < nThreads; t++ {
		start := t * chunkSize
		end := start + chunkSize
		if end > len(projectiles) {
			end = len(projectiles)
		}
		if start >= len(projectiles) {
			break
		}
		wg.Add(1)
		go func(sub []Projectile) {
			defer wg.Done()
			localProj := []Projectile{}
			localHits := []Hit{}
			for i := range sub {
				p := &sub[i]
				if !p.Alive || !p.Target.Alive {
					continue
				}
				p.Pos = p.Pos.Add(p.Vel.Scale(SimStep))
				p.FlightTime += SimStep
				if p.Pos.Sub(p.Target.Pos).Len() <= ShipRadius {
					localHits = append(localHits, Hit{Target: p.Target, Damage: p.Damage})
					p.Alive = false
					continue
				}
				if !InCube(p.Pos) {
					p.Alive = false
					continue
				}
				localProj = append(localProj, *p)
			}
			resChan <- result{newProj: localProj, hits: localHits}
		}(projectiles[start:end])
	}
	wg.Wait()
	close(resChan)
	newProj := []Projectile{}
	hits := []Hit{}
	for r := range resChan {
		newProj = append(newProj, r.newProj...)
		hits = append(hits, r.hits...)
	}
	return newProj, hits
}

// Применяем урон после всех расчётов (чтобы race condition не было)
func ApplyHits(hits []Hit) {
	for _, hit := range hits {
		if hit.Target.Alive && hit.Target.Armor > hit.Damage {
			hit.Target.Armor -= hit.Damage
		} else if hit.Target.Alive {
			hit.Target.Armor = 0
			hit.Target.Alive = false
		}
	}
}

func InCube(pos Vec3) bool {
	return pos.X >= 0 && pos.Y >= 0 && pos.Z >= 0 && pos.X <= CubeSize && pos.Y <= CubeSize && pos.Z <= CubeSize
}

func UpdateProjectiles3DBatched(projectiles []Projectile, batchSize int) ([]Projectile, []Hit) {
	type result struct {
		newProj []Projectile
		hits    []Hit
	}
	resChan := make(chan result, (len(projectiles)+batchSize-1)/batchSize)
	var wg sync.WaitGroup

	for i := 0; i < len(projectiles); i += batchSize {
		end := i + batchSize
		if end > len(projectiles) {
			end = len(projectiles)
		}
		wg.Add(1)
		go func(sub []Projectile) {
			defer wg.Done()
			localProj := make([]Projectile, 0, len(sub))
			localHits := make([]Hit, 0, 4)
			for j := range sub {
				p := &sub[j]
				if !p.Alive || !p.Target.Alive {
					continue
				}
				p.Pos = p.Pos.Add(p.Vel.Scale(SimStep))
				p.FlightTime += SimStep
				if p.Pos.Sub(p.Target.Pos).Len() <= ShipRadius {
					localHits = append(localHits, Hit{Target: p.Target, Damage: p.Damage})
					p.Alive = false
					continue
				}
				if !InCube(p.Pos) {
					p.Alive = false
					continue
				}
				localProj = append(localProj, *p)
			}
			resChan <- result{newProj: localProj, hits: localHits}
		}(projectiles[i:end])
	}

	wg.Wait()
	close(resChan)

	newProj := make([]Projectile, 0, len(projectiles))
	hits := []Hit{}
	for r := range resChan {
		newProj = append(newProj, r.newProj...)
		hits = append(hits, r.hits...)
	}
	return newProj, hits
}

func UpdateProjectiles3DWorkerPool(projectiles []Projectile, batchSize int) ([]Projectile, []Hit) {
	type job struct {
		batch []Projectile
	}
	type result struct {
		newProj []Projectile
		hits    []Hit
	}

	numWorkers := runtime.NumCPU() // или чуть больше
	jobs := make(chan job)
	results := make(chan result)

	// стартуем фиксированный пул воркеров
	for w := 0; w < numWorkers; w++ {
		go func() {
			for j := range jobs {
				localProj := make([]Projectile, 0, len(j.batch))
				localHits := make([]Hit, 0, 4)
				for i := range j.batch {
					p := &j.batch[i]
					if !p.Alive || !p.Target.Alive {
						continue
					}
					p.Pos = p.Pos.Add(p.Vel.Scale(SimStep))
					p.FlightTime += SimStep
					if p.Pos.Sub(p.Target.Pos).Len() <= ShipRadius {
						localHits = append(localHits, Hit{Target: p.Target, Damage: p.Damage})
						p.Alive = false
						continue
					}
					if !InCube(p.Pos) {
						p.Alive = false
						continue
					}
					localProj = append(localProj, *p)
				}
				results <- result{newProj: localProj, hits: localHits}
			}
		}()
	}

	go func() {
		for i := 0; i < len(projectiles); i += batchSize {
			end := i + batchSize
			if end > len(projectiles) {
				end = len(projectiles)
			}
			jobs <- job{batch: projectiles[i:end]}
		}
		close(jobs)
	}()

	newProj := make([]Projectile, 0, len(projectiles))
	hits := []Hit{}
	numBatches := (len(projectiles) + batchSize - 1) / batchSize
	for i := 0; i < numBatches; i++ {
		r := <-results
		newProj = append(newProj, r.newProj...)
		hits = append(hits, r.hits...)
	}
	return newProj, hits
}
