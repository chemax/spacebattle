package main

import (
	"fmt"
	"math"
	"math/rand"
)

const (
	G             = 9.81
	SimStep       = 0.02
	MaxTime       = 600
	MaxAccel      = 0.2 * G
	ShipRadius    = 500.0
	CubeSize      = 1e5 // 100 000 км в метрах
	Pellets       = 3200
	SpreadAngle   = math.Pi / 3 // 45°
	EffectiveDist = 2e7         // 20 000 км
)

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
	// Случайный угол вокруг оси (Гауссов разброс в пределах spread)
	theta := rand.NormFloat64() * (spread / 2)
	phi := rand.Float64() * 2 * math.Pi
	// Сферические координаты — новый вектор
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

func main() {
	ship1 := Ship{
		Name:    "Orion",
		Pos:     Vec3{0, 0, 0},
		Vel:     Vec3{0, 0, 0},
		Armor:   300,
		Guns:    []Gun{{ROF: 1, ProjectileV: 2500, Damage: 80}},
		Alive:   true,
		Evasion: MaxAccel,
	}
	ship2 := Ship{
		Name:    "Valkyrie",
		Pos:     Vec3{CubeSize, CubeSize, CubeSize},
		Vel:     Vec3{0, 0, 0},
		Armor:   300,
		Guns:    []Gun{{ROF: 1, ProjectileV: 2500, Damage: 80}},
		Alive:   true,
		Evasion: MaxAccel,
	}
	ship1.Target = &ship2
	ship2.Target = &ship1

	projectiles := []Projectile{}
	t := 0.0

	for ; t < MaxTime && ship1.Alive && ship2.Alive; t += SimStep {
		SmartManeuver3D(&ship1, projectiles)
		SmartManeuver3D(&ship2, projectiles)
		UpdatePhysics3D(&ship1)
		UpdatePhysics3D(&ship2)
		FireShotgun3D(&ship1, &ship2, &projectiles, Pellets, SpreadAngle)
		FireShotgun3D(&ship2, &ship1, &projectiles, Pellets, SpreadAngle)
		projectiles = UpdateProjectiles3D(projectiles)
		if int(t*10)%100 == 0 {
			fmt.Printf("t=%.0fs: %s(%.1f) <-> %s(%.1f), dist=%.1fкм, активных пуль: %d\n",
				t, ship1.Name, ship1.Armor, ship2.Name, ship2.Armor, ship1.Pos.Sub(ship2.Pos).Len()/1000, len(projectiles))
		}
	}
	fmt.Printf("\nФИНАЛ:\n")
	fmt.Printf("%s: броня %.1f, живой: %v\n", ship1.Name, ship1.Armor, ship1.Alive)
	fmt.Printf("%s: броня %.1f, живой: %v\n", ship2.Name, ship2.Armor, ship2.Alive)
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
		// расстояние до траектории (через векторное произведение)
		cross := Vec3{
			relPos.Y*relVel.Z - relPos.Z*relVel.Y,
			relPos.Z*relVel.X - relPos.X*relVel.Z,
			relPos.X*relVel.Y - relPos.Y*relVel.X,
		}
		distToTraj := cross.Len() / relVel.Len()
		if maxOffset > distToTraj-ShipRadius {
			// Уклоняемся по случайному ортогональному направлению
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

// 3D обновление позиции
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
				// 3D рассеивание (любой рандомный угол в пределах SpreadAngle)
				spreadDir := Random3DSpread(dir, spreadAngle)
				projVel := spreadDir.Scale(gun.ProjectileV).Add(attacker.Vel)
				*projectiles = append(*projectiles, Projectile{
					Pos:    attacker.Pos,
					Vel:    projVel,
					Target: defender,
					Damage: gun.Damage / float64(nPellets),
					Alive:  true,
				})
			}
		}
	}
}

// 3D рассеивание вокруг основного направления
func Random3DSpread(dir Vec3, angle float64) Vec3 {
	// Ортонормированный базис: dir, ortho1, ortho2
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
	// Смещаем на угол theta от dir в случайном направлении
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

// Апдейт снарядов и попадания/уничтожение вышедших за пределы куба
func UpdateProjectiles3D(projectiles []Projectile) []Projectile {
	newProj := projectiles[:0]
	for i := range projectiles {
		p := &projectiles[i]
		if !p.Alive || !p.Target.Alive {
			continue
		}
		p.Pos = p.Pos.Add(p.Vel.Scale(SimStep))
		p.FlightTime += SimStep
		if p.Pos.Sub(p.Target.Pos).Len() <= ShipRadius {
			ApplyDamage3D(p.Target, p.Damage)
			p.Alive = false
			continue
		}
		// Удалить если вышел за пределы куба (0...CubeSize)
		if !InCube(p.Pos) {
			p.Alive = false
			continue
		}
		newProj = append(newProj, *p)
	}
	return newProj
}

func InCube(pos Vec3) bool {
	return pos.X >= 0 && pos.Y >= 0 && pos.Z >= 0 && pos.X <= CubeSize && pos.Y <= CubeSize && pos.Z <= CubeSize
}

func ApplyDamage3D(ship *Ship, dmg float64) {
	if ship.Armor > dmg {
		ship.Armor -= dmg
	} else {
		ship.Armor = 0
		ship.Alive = false
	}
}
